#include "stm32f10x.h"                
#include "stm32f10x_exti.h"           
#include "stm32f10x_gpio.h"           
#include "stm32f10x_usart.h"         
#include "stm32f10x_rcc.h"            
#include "stm32f10x_dma.h"            
#include "lcd.h"                      
#include "touch.h"                    
#include "misc.h"                    
#include <stdbool.h>                 

/* 조도 및 온도 센서를 기반으로 블라인드 상태를 제어하는 스마트 블라인드 시스템 
수요일 분반 5조
팀장: 김대욱
팀원: 설종환, 김정민, 이승원
*/

#define COMMAND_QUEUE_SIZE 500       // 명령 큐의 크기 정의. 500 바이트
#define TEMP_THRESHOLD 28             // 온도 임계값(28 이상이면 블라인드 작동)
#define LIGHT_THRESHOLD 4000         // 조도 임계값(4000 이상이면 온도 센서 활성화)
#define BLIND_UP_PERIOD 700000 // 블라인드 올림 작동 시간
#define BLIND_DOWN_PERIOD 500000 // 블라인드  내림 작동 시간

/* 블라인드 상태를 표현하기 위한 FSM 구조 */
volatile enum {
    BLIND_POSITION_BOTTOM,  // 블라인드가 내려간 상태
    BLIND_POSITION_MOVING, // 블라인드가 이동 중
    BLIND_POSITION_TOP     // 블라인드가 올라간 상태
} blindPosition = BLIND_POSITION_BOTTOM; // 초기 상태는 내려간 상태로 정의

/* 전역 변수 선언 */
/* 큐를 사용하여 블루투스 명령 누락 방지 및 인터럽트와 메인 루프 작업 분리 */
volatile char commandQueue[COMMAND_QUEUE_SIZE];  // 명령 저장용 큐
volatile int queueHead = 0;                      // 큐의 헤드 위치
volatile int queueTail = 0;                      // 큐의 테일 위치
volatile bool tempSensorActive = false;          // 온도 센서 활성화 플래그

volatile uint32_t ADC_Values[2];                 // ADC 변환 결과 저장 (0: 조도, 1: 온도)
char flagUART1 = 0;                              // UART1 데이터 수신 플래그
char flagUART2 = 0;                              // UART2 데이터 수신 플래그
char commandFromUART1;                           // UART1 (PC)로부터 수신된 명령
char commandFromUART2;                           // UART2 (블루투스)로부터 수신된 명령
char stateString[10] = "";                      // LCD에 표시할 상태 문자열

/* 함수 원형 선언 */
void RCC_Configure(void);                        // 클럭 설정
void GPIO_Configure(void);                       // GPIO 핀 설정
void USART1_Init(void);                          // UART1 초기화
void USART2_Init(void);                          // UART2 초기화
void NVIC_Configure(void);                       // 인터럽트 설정
void ADC_Configure(void);                        // ADC 초기화 및 설정

/* 블라인드 동작 관련 함수 */
void Motor_Control(char command);                // 모터 제어 (블라인드 동작)
void Blind_Up(void);                             // 블라인드 올리기
void Blind_Down(void);                           // 블라인드 내리기
void Blind_Stop(void);                           // 블라인드 정지

/* 유틸리티 함수 */
void delay(uint32_t delayTime);                  // 소프트웨어 지연 함수
void sendDataUART1(uint16_t data);               // UART1 데이터 송신
void sendDataUART2(uint16_t data);               // UART2 데이터 송신
void showLightAndTempValue(void);                // 조도 및 온도 값 표시

// 지연 함수
void delay(uint32_t delayTime) {
    for (volatile uint32_t i = 0; i < delayTime; i++);  // 단순 루프를 이용한 지연
}

/* 
 * 명령을 큐에 추가
 * 새로운 명령을 큐에 추가하여 인터럽트와 메인 루프 간 데이터 손실을 방지
 * 큐가 가득 찬 경우, "Full Queue" 상태를 LCD에 표시
 */
void enqueueCommand(char command) {
    int nextTail = (queueTail + 1) % COMMAND_QUEUE_SIZE; // 큐가 순환 구조를 따르도록 모듈러 연산 사용
    if (nextTail == queueHead) {                        // 큐가 가득 찬 경우
        sprintf(stateString, "Full Queue");             // 상태 문자열 설정
        return;
    }
    commandQueue[queueTail] = command;                  // 큐에 명령 추가
    queueTail = nextTail;                               // 테일 인덱스 갱신
}

/* 큐에서 명령을 가져옴 */
char dequeueCommand(void) {
    if (queueHead == queueTail) {                       // 큐가 비어 있는 경우
        return 0;                                       // 0 반환
    }
    char command = commandQueue[queueHead];             // 헤드 위치의 명령 가져오기
    queueHead = (queueHead + 1) % COMMAND_QUEUE_SIZE;   // 헤드 위치 갱신
    return command;
}

/* 클럭 설정 */
void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIOA 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // GPIOB 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // GPIOC 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // GPIOD 클럭 활성화

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE); // USART1 클럭 활성화
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE); // USART2 클럭 활성화

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // AFIO 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  // ADC1 클럭 활성화
}

/* GPIO 설정 */
void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // UART1 TX(PA9), RX(PA10) 핀 설정
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           // TX 핀
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 핀 속도 50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // AF 출력
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          // RX 핀
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       // 풀업 입력
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // UART2 TX(PA2), RX(PA3) 핀 설정
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;           // TX 핀
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // AF 출력
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;           // RX 핀
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       // 풀업 입력
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 모터1(PB1, PB2) 핀 설정
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 출력
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 핀 속도 50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 모터2(PD1, PD2) 핀 설정
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 출력
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 조도 센서(PC2), 온도 센서(PC1) 핀 설정
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;       // 아날로그 입력
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* 
* ADC 설정
* 조도 센서와 온도 센서는 동일한 ADC를 공유하며 채널 변경을 통해 데이터를 수집 
*/
void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;

    ADC_DeInit(ADC1); // ADC 초기화
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // 독립 모드 설정
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // 단일 채널 모드
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 소프트웨어 트리거 모드
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;             // 단일 채널
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5); // 초기 채널: 조도 센서
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);            // ADC 변환 완료 인터럽트 활성화
    ADC_Cmd(ADC1, ENABLE);                             // ADC 활성화
    
    // ADC 보정
    ADC_ResetCalibration(ADC1);               // 보정 초기화
    while (ADC_GetResetCalibrationStatus(ADC1)); // 보정 초기화 완료 대기
    ADC_StartCalibration(ADC1);               // 보정 시작
    while (ADC_GetCalibrationStatus(ADC1));   // 보정 완료 대기
}

/* 
* NVIC 설정 
* USART는 원활한 실시간 통신을 보장하기 위해 높은 우선순위를 갖도록 설정
*/
void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     // 우선순위 그룹 설정

    // ADC 인터럽트: 낮은 우선순위
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 낮은 우선순위
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART1 인터럽트: 높은 우선순위
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 높은 우선순위
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    // USART2 인터럽트: 높은 우선순위
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 높은 우선순위
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
}

/* USART1 초기화 */
void USART1_Init(void) {
    USART_InitTypeDef USART1_InitStructure;

    // USART1 활성화
    USART_Cmd(USART1, ENABLE);

    // USART1 설정
    USART1_InitStructure.USART_BaudRate = 9600;                 // 통신 속도 설정
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b; // 데이터 길이 (8비트)
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // RX, TX 활성화
    USART1_InitStructure.USART_Parity = USART_Parity_No;        // 패리티 비트 없음
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;     // 1비트 스톱 비트
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 흐름 제어 없음

    USART_Init(USART1, &USART1_InitStructure);                  // USART1 설정 적용
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);              // RX 인터럽트 활성화
}

/* USART2 초기화 */
void USART2_Init(void) {
    USART_InitTypeDef USART2_InitStructure;

    // USART2 활성화
    USART_Cmd(USART2, ENABLE);

    // USART2 설정
    USART2_InitStructure.USART_BaudRate = 9600;                 // 통신 속도 설정
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b; // 데이터 길이 (8비트)
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // RX, TX 활성화
    USART2_InitStructure.USART_Parity = USART_Parity_No;        // 패리티 비트 없음
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;     // 1비트 스톱 비트
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 흐름 제어 없음

    USART_Init(USART2, &USART2_InitStructure);                  // USART2 설정 적용
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);              // RX 인터럽트 활성화
}

/* USART1 인터럽트 핸들러 */
void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        // 데이터 수신 완료 플래그 확인
        flagUART1 = 1;
        commandFromUART1 = USART_ReceiveData(USART1); // 데이터 수신
        enqueueCommand(commandFromUART1);            // 명령 큐에 추가
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); // 인터럽트 플래그 클리어
    }
}

/* USART2 인터럽트 핸들러 */
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        // 데이터 수신 완료 플래그 확인
        flagUART2 = 1;
        commandFromUART2 = USART_ReceiveData(USART2); // 데이터 수신
        enqueueCommand(commandFromUART2);            // 명령 큐에 추가
        USART_ClearITPendingBit(USART2, USART_IT_RXNE); // 인터럽트 플래그 클리어
    }
}

/* 
* ADC 인터럽트 핸들러 
* 조도 센서 또는 온도 센서의 값을 읽어 전역 변수 ADC_Values[2]에 각각 저장하고, 조건에 따라 블라인드를 제어
*/
void ADC1_2_IRQHandler(void) {
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
        uint32_t adcValue = ADC_GetConversionValue(ADC1);

        if (!tempSensorActive) {
            // 조도 센서 값 읽기
            ADC_Values[0] = adcValue;

            if (ADC_Values[0] > LIGHT_THRESHOLD) {
                // 조도 값이 임계값(4000) 이상이면, ADC채널이 온도 센서 값을 읽도록 채널 전환
                tempSensorActive = true;
                ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5);

                // 채널 전환 후 쓰레기 값을 버리기 위해 첫 번째 데이터는 무시
                ADC_SoftwareStartConvCmd(ADC1, ENABLE);
                while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // 변환 완료 대기
                (void)ADC_GetConversionValue(ADC1); // 첫 번째 값 버림

                // 채널 변환 후 보정
                ADC_ResetCalibration(ADC1);
                while (ADC_GetResetCalibrationStatus(ADC1));
                ADC_StartCalibration(ADC1);
                while (ADC_GetCalibrationStatus(ADC1));
            }
        } else {
            // 온도 센서 값 읽기
            ADC_Values[1] = adcValue;

            // 온도 계산
            float temperatureCelsius = (ADC_Values[1] * 3.3 / 4096) * 100;

            if (temperatureCelsius >= TEMP_THRESHOLD) {
                // 온도가 임계값 이상이면 블라인드 내리기
                Blind_Down();
            }

            // ADC채널이 조도 센서 값을 읽도록 채널 전환
            tempSensorActive = false;
            ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5);

            // 채널 전환 후 쓰레기 값을 버리기 위해 첫 번째 데이터는 무시
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // 변환 완료 대기
            (void)ADC_GetConversionValue(ADC1); // 첫 번째 값 버림

            // 채널 변환 후 보정
            ADC_ResetCalibration(ADC1);
            while (ADC_GetResetCalibrationStatus(ADC1));
            ADC_StartCalibration(ADC1);
            while (ADC_GetCalibrationStatus(ADC1));
        }

        // 다음 변환 시작
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        // 인터럽트 플래그 클리어
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

/* 
 * 모터 제어 함수
 * 휴대폰을 통해 USART2에 송신한 명령에 따라 모터를 제어하여 블라인드 상태를 변경
 */
void Motor_Control(char command) {
    switch (command) {   
        case 'u': // 블라인드 올리기
            Blind_Up();
            break;
        case 'd': // 블라인드 내리기
            Blind_Down();
            break;
        case 's': // 블라인드 동작 정지
            Blind_Stop();
            break;
        default: // 잘못된 명령
            break;
    }
}

/* 블라인드 올리는 동작 */
void Blind_Up(void) {
    if (blindPosition == BLIND_POSITION_TOP) {
        sprintf(stateString, "Already Up");
        return; // 블라인드가 이미 올라간 상태면 동작하지 않음
    }

    blindPosition = BLIND_POSITION_MOVING; // 이동 중 상태로 설정

    // 모터1 정방향 
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);  
    GPIO_SetBits(GPIOB, GPIO_Pin_2);

    // 모터2 역방향
    GPIO_ResetBits(GPIOD, GPIO_Pin_2);  
    GPIO_SetBits(GPIOD, GPIO_Pin_1); 
    sprintf(stateString, "Blind Up");

    delay(BLIND_UP_PERIOD); 

    // 블라인드 다 올리면 멈추기
    Blind_Stop();

    blindPosition = BLIND_POSITION_TOP; // 최상단 상태로 설정
}

/* 블라인드 내리는 동작 */
void Blind_Down(void) {
    if (blindPosition == BLIND_POSITION_BOTTOM) {
        sprintf(stateString, "Already Down");
        return; // 블라인드가 이미 내려간 상태면 동작하지 않음
    }

    blindPosition = BLIND_POSITION_MOVING; // 이동 중 상태로 설정

    // 모터1 역방향
    GPIO_ResetBits(GPIOB, GPIO_Pin_2); 
    GPIO_SetBits(GPIOB, GPIO_Pin_1);

    // 모터2 정방향
    GPIO_ResetBits(GPIOD, GPIO_Pin_1);  
    GPIO_SetBits(GPIOD, GPIO_Pin_2); 
    sprintf(stateString, "Blind Down");

    delay(BLIND_DOWN_PERIOD); 

    // 블라인드 다 내리면 멈추기
    Blind_Stop();

    blindPosition = BLIND_POSITION_BOTTOM; // 최하단 상태로 설정
}

/* 블라인드 멈추는 동작 */
void Blind_Stop(void) {
    GPIO_SetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_2);
    GPIO_SetBits(GPIOD, GPIO_Pin_1 | GPIO_Pin_2);

    blindPosition = BLIND_POSITION_MOVING; // 이동 중 상태에서 정지로 변경
}

/* UART1 데이터 송신 */
void sendDataUART1(uint16_t data) {
    /* Wait till TC is set */
    USART_SendData(USART1, data);
}

/* UART2 데이터 송신 */
void sendDataUART2(uint16_t data) {
    /* Wait till TC is set */
    USART_SendData(USART2, data);
}

/* LCD에 온도 및 조도 값 출력 */
void showLightAndTempValue(void) {
    char buffer[32];

    // 조도 센서 값 출력
    sprintf(buffer, "Light: %4lu", ADC_Values[0]);
    LCD_ShowString(10, 10, buffer, BLACK, WHITE);

    // 온도 센서 값 출력
    sprintf(buffer, "Raw Temp: %4lu", ADC_Values[1]);
    LCD_ShowString(10, 30, buffer, BLACK, WHITE);

    // 변환된 온도 출력
    float temperatureCelsius = (ADC_Values[1] * 3.3 / 4096) * 100;
    sprintf(buffer, "Temp: %2.1f C", temperatureCelsius);
    LCD_ShowString(10, 50, buffer, BLACK, WHITE);
}

int main(void) {
    // 시스템 초기화 (클럭 설정 및 기본 시스템 설정)
    SystemInit();

    // 주변 장치 초기화
    RCC_Configure();          // 클럭 설정
    GPIO_Configure();         // GPIO 핀 설정
    ADC_Configure();          // ADC 설정

    USART1_Init();            // USART1 초기화 (PC와의 통신)
    USART2_Init();            // USART2 초기화 (블루투스 통신)
    NVIC_Configure();         // NVIC 인터럽트 설정
    LCD_Init();               // LCD 초기화
    LCD_Clear(WHITE);         // LCD 화면 초기화 (배경 색상: 흰색)

    // ADC 변환 시작 (최초 소프트웨어 트리거)
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while (1) {
        // USART1 명령 처리
        if (flagUART1 == 1) {
            // USART1에서 명령 수신 시 처리
            char command;

            // 명령 큐에서 모든 명령 처리
            while ((command = dequeueCommand()) != 0) {
                sendDataUART2(command); // 수신한 명령을 USART2로 전달 (블루투스 장치로 전송)
            }
            
            // 명령 처리가 끝나면 플래그 리셋
            flagUART1 = 0;
        }
        
        // USART2 명령 처리
        if (flagUART2 == 1) {
            // USART2에서 명령 수신 시 처리
            char command;

            // 명령 큐에서 모든 명령 처리
            while ((command = dequeueCommand()) != 0) {
                sendDataUART1(command);  // 수신한 명령을 USART1으로 전달 (PC로 전송)
                Motor_Control(command); // 명령에 따라 모터 제어 (블라인드 조작)
                LCD_ShowString(100, 100, stateString, BLACK, WHITE); // LCD에 현재 상태 표시
            }

            // 명령 처리가 끝나면 플래그 리셋
            flagUART2 = 0;
        }

        // ADC 센서 값 표시 (조도 값 및 온도 값)
        showLightAndTempValue();
    }
}