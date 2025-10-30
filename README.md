## 📝 스마트 블라인드 시스템 텀 프로젝트 결과 보고서

[cite_start]**과목명:** 임베디드 시스템 설계 및 실험 [cite: 6, 7, 8]
[cite_start]**담당교수:** 김원석 [cite: 9]
[cite_start]**제출일:** 2024/12/23 [cite: 10]
[cite_start]**학과:** 정보컴퓨터공학부 [cite: 11]
[cite_start]**팀장:** 김대욱 [cite: 12]
[cite_start]**팀원:** 설종환, 박정민, 이승원 [cite: 14, 15]

---

### 1. 프로젝트 개요 (Overview)

[cite_start]본 프로젝트는 **스마트 블라인드 시스템**을 개발하여 현대 가정 및 사무실 환경에서 편리성, 에너지 절약, 사용자 경험 개선을 목표로 합니다[cite: 32]. [cite_start]여러 센서와 블루투스 모듈을 활용하여 환경 조건이나 사용자의 원격 명령에 따라 블라인드를 자동으로 제어하는 임베디드 시스템을 구현했습니다[cite: 34, 35, 36].

---

### 2. 주요 개발 내용 및 기술 요약 (Key Features and Technology Summary)

#### 2.1. 기능 구현
| 기능 | 구현 방식 | 설명 |
|:---|:---|:---|
| **환경 센서 기반 자동 제어** | [cite_start]ADC1 및 두 채널(조도, 온도) 기반 데이터 변환 [cite: 96] | [cite_start]**조도 센서 (GL5537)**로 주변 광량을 측정하며, 임계값 (4000 lux)을 초과할 경우에만 **온도 센서 (LM35DZ)**를 활성화하여 에너지 소비를 절감합니다[cite: 38, 39, 40, 41]. [cite_start]온도가 임계값 (28도) 이상이면 블라인드가 자동으로 내려갑니다[cite: 42]. |
| **블루투스 및 유선 수동 제어** | [cite_start]UART2/UART1 인터럽트를 활용한 명령 수신 및 처리 [cite: 96] | [cite_start]**블루투스 (UART2)** 모듈을 통해 스마트폰 앱으로 원격 제어가 가능하며, **유선 (UART1)** 명령을 통해 PC에서도 제어할 수 있습니다[cite: 44, 45, 50]. [cite_start]명령은 'u'(올리기), 'd'(내리기), 's'(정지)로 구성됩니다[cite: 47, 48, 49]. |
| **모터 제어 및 안전 장치** | [cite_start]GPIO 기반 모터 방향 및 속도 제어 [cite: 96] | [cite_start]모터 2개와 L298N 모터 드라이브를 사용하여 블라인드의 상하를 제어합니다[cite: 60]. [cite_start]블라인드가 이미 최상단/최하단에 있는 경우 추가 명령을 무시합니다[cite: 62, 63]. |
| **명령 처리 안정화** | [cite_start]고정 크기의 큐(command Queue)로 명령 처리 안정화 [cite: 96] | [cite_start]명령 큐를 사용하여 다수의 명령이 동시에 입력되어도 안정적으로 처리하며, UART 통신에서 명령 무시 현상을 해결했습니다[cite: 51, 158, 160]. |
| **LCD 디스플레이 정보 제공** | [cite_start]LCD\_ShowString 함수로 상태와 데이터를 실시간 표시 [cite: 96] | [cite_start]블라인드 상태 ("blind up", "blind down", "stopped"), 현재 조도/온도 데이터, 그리고 에러 메시지 ("Full Queue", "Already Up", "Already Down")를 실시간으로 표시합니다[cite: 55, 56, 57, 58]. |

#### 2.2. 하드웨어 설정 (Configuration Summary)
* [cite_start]**클럭 설정 (RCC):** GPIOA, B, C, D 포트, USART1/2, AFIO, ADC1에 클럭 활성화[cite: 162, 173, 175].
* **GPIO 설정:**
    * [cite_start]UART1 (TX: PA9, RX: PA10), UART2 (TX: PA2, RX: PA3) 핀 설정[cite: 179, 187].
    * [cite_start]모터 1 (PB1, PB2), 모터 2 (PD1, PD2) 핀을 Push-Pull Output으로 설정[cite: 194, 200].
    * [cite_start]조도 센서 (PC2), 온도 센서 (PC1) 핀을 아날로그 입력 (AIN)으로 설정[cite: 202, 205].
* [cite_start]**ADC 설정:** ADC1을 독립 모드, 단일 채널 모드로 설정하고, EOC(변환 완료) 인터럽트 활성화[cite: 215, 217, 221, 226]. [cite_start]초기 채널은 조도 센서 (ADC\_Channel\_12)로 설정[cite: 225].
* [cite_start]**USART 설정:** USART1 및 USART2 모두 BaudRate 9600, 8비트 워드 길이, 패리티 없음, 1 Stop Bit로 설정되었으며, RXNE 인터럽트 활성화[cite: 274, 276, 279, 284, 289, 298, 299].
* [cite_start]**NVIC (인터럽트 우선순위):** USART1 및 USART2를 ADC보다 높은 우선순위로 설정하여 실시간 통신을 보장합니다[cite: 258, 263, 267, 268].

---

### 3. 주요 알고리즘 및 코드 분석

#### 3.1. 명령 큐 (Command Queue)
[cite_start]UART 통신 중 명령이 연속적으로 들어올 때 발생하는 무시 현상을 방지하기 위해 고정 크기의 큐를 사용했습니다[cite: 158, 159, 160].
* `enqueueCommand(char command)`: 명령을 큐의 꼬리(`queueTail`)에 저장합니다. [cite_start]큐가 가득 차면 "Full Queue" 메시지를 설정하고 명령 추가를 거부합니다[cite: 135, 137, 138].
* `dequeueCommand(void)`: 큐의 머리(`queueHead`)에서 명령을 꺼냅니다. [cite_start]큐가 비어 있으면 0을 반환합니다[cite: 147, 148, 150].

#### 3.2. 인터럽트 핸들러 (Interrupt Handlers)
* [cite_start]`USART1_IRQHandler()` / `USART2_IRQHandler()`: 데이터 수신 시 (RXNE 인터럽트 발생) 해당 플래그를 설정하고, 수신된 명령을 **명령 큐**에 저장합니다[cite: 303, 307, 312, 314, 319].
* `ADC1_2_IRQHandler()`:
    * [cite_start]ADC 변환 완료 시 (EOC 인터럽트 발생) 센서값을 추출합니다[cite: 322, 323].
    * [cite_start]**조도 센서** 값이 임계값 (LIGHT\_THRESHOLD)을 넘으면 **온도 센서 (채널 11)**로 전환하고, 첫 번째 부정확한 변환값을 버린 후 보정(Calibration)을 수행하여 활성화합니다[cite: 330, 331, 332, 340, 343].
    * [cite_start]**온도 센서** 값이 임계값 (TEMP\_THRESHOLD) 이상이면 `Blind_Down()` 함수를 호출하고, 다시 **조도 센서 (채널 12)**로 전환합니다[cite: 358, 360, 361, 363].

#### 3.3. 모터 제어 (Motor Control)
* [cite_start]`Motor_Control(char command)`: 'u', 'd', 's' 명령에 따라 `Blind_Up()`, `Blind_Down()`, `Blind_Stop()` 함수를 호출합니다[cite: 386, 388, 402, 403].
* [cite_start]`Blind_Up()`, `Blind_Down()`: `blindPosition` (FSM 구조)을 확인하여 이미 최상단/최하단일 경우 동작을 무시하고 메시지를 표시합니다[cite: 405, 429]. [cite_start]이동 중 상태로 설정하고, 모터 1/2의 GPIO 비트(IN1~IN4)를 설정하여 정방향/역방향으로 회전시키며, 설정된 시간(`BLIND_UP_PERIOD`/`BLIND_DOWN_PERIOD`)만큼 `delay()`를 준 후 `Blind_Stop()`을 호출합니다[cite: 410, 411, 414, 434, 435, 438].
    * [cite_start]**⚠️ 기존 계획 대비 변경 사항:** PWM 제어를 통한 모터 속도 조절을 시도했으나 핀 문제로 인해 실패하여, GPIO 제어 기반의 최대 속도 회전으로 구현되었습니다[cite: 70, 71, 72, 73, 424].
* [cite_start]`Blind_Stop()`: 모든 모터 제어 핀의 비트를 Set하여 모터 드라이브의 **브레이크 모드**를 활성화하고 블라인드 이동을 정지합니다[cite: 444, 445, 448].

---

### 4. 시나리오 (Execution Scenarios)

#### 4.1. 자동 제어 시나리오
1.  조도 센서가 광량을 측정합니다. [cite_start]조도 > 4000 lux이면 온도 센서가 활성화됩니다[cite: 78].
2.  온도 센서가 온도를 측정합니다. [cite_start]온도 $\ge 28^\circ \text{C}$이면 블라인드가 자동으로 내려가고 LCD에 "**Blind Down**"이 표시됩니다[cite: 79, 80].

#### 4.2. 수동 제어 시나리오
1.  [cite_start]스마트폰 앱에서 'u' 명령 전송 $\rightarrow$ 블라인드 상승, LCD에 "**Blind Up**" 표시[cite: 82].
2.  [cite_start]'d' 명령 전송 $\rightarrow$ 블라인드 하강, LCD에 "**Blind Down**" 표시[cite: 83].
3.  [cite_start]'s' 명령 전송 $\rightarrow$ 블라인드 동작 멈춤, LCD에 "**Stopped**" 표시[cite: 84].
4.  [cite_start]최상단/최하단 상태에서 추가 명령 입력 $\rightarrow$ 모터 미작동, LCD에 "**Already Up**" 또는 "**Already Down**" 표시[cite: 85, 86].

#### 4.3. 에러 처리 시나리오
1.  [cite_start]명령 큐가 가득 찬 경우 $\rightarrow$ LCD에 "**Full Queue**" 메시지 표시, 추가 명령 대기 상태로 전환[cite: 88].
2.  [cite_start]UART로 잘못된 명령 전송 $\rightarrow$ 명령 무시, LCD에 아무 메시지도 출력되지 않음[cite: 89].
