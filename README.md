# 🎹 STM32 Bare-Metal Piano & Recorder
<img src="https://github.com/user-attachments/assets/cda15dc8-6302-41ee-9eac-1bb0deebb958" width="400">><div>
**MCU 프로세서 구조 및 디바이스 프로그래밍 최종 프로젝트**
> 
> 본 프로젝트는 AI의 도움이나 외부 라이브러리 의존 없이, **STM32 Reference Manual을 직접 분석하여 Register Level에서 하드웨어를 제어**한 임베디드 시스템 프로젝트입니다.

---

## 🚀 Project Overview
STM32 MCU의 타이머(Timer), GPIO 구조를 깊이 있게 이해하고, 이를 활용해 실시간 음계 생성 및 **CNT 레지스터 기반의 정밀한 녹음/재생 시스템**을 구현하는 것을 목표로 합니다.

* **Development Environment:** STM32CubeIDE (Bare-Metal & HAL)
* **Hardware:** STM32 Development Board, Tactile Switches (8ea), Passive Buzzer, Resistors
* **Key Achievement:** 하드웨어 타이머 레지스터 직접 제어를 통한 실시간 오디오 합성 및 데이터 로깅

---

## 🛠 Tech Stack & Implementation Details

### 1. 하드웨어 레벨 소리 제어 (AI-Free, Pure Logic)
데이터시트의 타이머 블록 다이어그램을 분석하여 PWM 신호를 생성했습니다.

* **주파수 제어 (Audio Synthesis):**
    * **PSC (Prescaler) & ARR (Auto-Reload Register):** 시스템 클럭을 분주하여 각 음계(C4~C5)에 해당하는 정밀한 주파수를 설정했습니다.
    * **CCR (Capture Compare Register):** Duty Cycle을 50%로 고정하여 일정한 음압의 사운드를 출력했습니다.
    * **수식 기반 설계:** $$f_{out} = \frac{f_{clk}}{(PSC + 1) \times (ARR + 1)}$$
    위 공식을 바탕으로 각 음계별 레지스터 설정값을 직접 계산하여 적용했습니다.



### 2. 입력 로직 및 안정화 (Input Strategy)
* **Polled GPIO:** 건반 입력의 동시성 확인 및 안정적인 상태 스캔을 위해 `HAL_GPIO_ReadPin` 기반의 폴링 방식을 채택했습니다.
* **Software Debouncing:** 스위치 누름 시 발생하는 채터링(Chattering) 현상을 소프트웨어적으로 처리하여 오작동 없는 깔끔한 연주가 가능하게 했습니다.

### 3. CNT 레지스터 기반 녹음 및 재생 (Recording System)
이 프로젝트의 핵심 기술로, 별도의 RTC 없이 **Timer의 CNT(Counter) 레지스터 값의 변화량**을 추적하여 구현했습니다.

* **녹음 원리:** 1. 건반 Press 시점의 `TIMx->CNT` 값 저장
    2. 건반 Release 시점의 `TIMx->CNT` 값 저장
    3. 두 값의 차(Delta)를 계산하여 배열에 저장 (음의 길이 측정)
* **재생 로직:** 저장된 시간 간격 데이터와 음계 정보를 매칭하여 연주한 리듬과 선율을 그대로 복원합니다. 내장 클럭의 카운팅을 활용했기에 박자 오차를 최소화했습니다.



---

## 💻 Key Features
* **8-Key Piano:** C4(도)부터 C5(도)까지 8개 건반 연주 기능
* **Real-time Recording:** 사용자가 연주하는 음표의 종류와 지속 시간을 실시간 배열에 저장
* **Playback Mode:** 기록된 데이터를 바탕으로 원곡의 템포를 살린 자동 재생 기능

---

## 📂 Project Structure
```text
├── Drivers/           # STM32 CMSIS & HAL Drivers
├── Core/
│   ├── Src/
│   │   ├── main.c     # PWM 주파수 계산 및 녹음/재생 핵심 로직
│   │   └── stm32xx_it.c # 타이머 인터럽트 핸들러 관리
│   └── Inc/           # 프로젝트 헤더 및 상수 정의
└── README.md

