# UNITA 2025.12.26 1차 기본교육

본 교육은 본격적인 ROS 2 학습에 앞서 **ROS 2 기반 자율주행 시스템의 전체 구조를 이해**하고,  
시뮬레이션 환경에서 구현 가능한 **간단한 자율주행 실습을 통해 흥미와 동기 부여를 강화**하는 것을 목표로 한다.

특히 자율주행 구현에 활용되는 **인지, 판단, 제어, 시뮬레이션** 등 다양한 기술 요소를  
직접 다뤄봄으로써 이후 심화 학습을 위한 기초 역량을 확보하고자 한다.

---
## 시뮬레이션 영상 (YouTube)

아래 썸네일을 클릭하면 실제 주행 영상을 확인할 수 있습니다.

[![Simulation Video](http://img.youtube.com/vi/2VR7mfooOVU/0.jpg)](https://www.youtube.com/watch?v=2VR7mfooOVU)

---

## 참여 인원 및 일시 & 장소

- **강의자**: 김형진  
- **교육 조교**: 이기현, 이다빈, 윤제호  
- **교육 참여자**:  
  강민수, 김민서, 박근호, 성현영, 윤지윤, 이석빈,  
  이원종, 장동혁, 정가용, 정규민, 한주형  

- **교육 일시**: 2025년 12월 26일 10:00 ~ 17:00  
  (점심시간 제외, 총 6시간)  
- **교육 장소**: 공과대학 8호관 117호  

---

### 0. 팀 구성

- **1팀**: 성현영, 정가용, 이석빈, 이다빈  
- **2팀**: 장동혁, 이원종, 윤제호  
- **3팀**: 윤지윤, 박근호, 정규민, 김형진  
- **4팀**: 한주형, 김민서, 강민수, 이기현  

---

### 1. 개발 환경 세팅 확인

- **운영체제**: Ubuntu 22.04 *(사전 설치 공지)*  
- **ROS 2**: Humble Hawksbill *(사전 설치 공지)*  
- **그래픽 드라이버**: NVIDIA Graphic Driver *(필요 시)*  

#### 실습 코드 링크

- **실습 코드 (실차/기본)**  
  https://github.com/SKKUAutoLab/H-Mobility-Autonomous-Advanced-Course

- **실습 코드 (시뮬레이션 ver.)**  
  https://github.com/SKKUAutoLab/H-Mobility-Autonomous-Advanced-Course-Simulation

```bash
# 본인이 받고자 하는 GitHub 레포지토리 클론
git clone <repository_url>

# 예시
git clone https://github.com/SKKUAutoLab/H-Mobility-Autonomous-Advanced-Course.git
```
두 개의 실습 코드를 모두 홈 디렉터리(~)에 설치한다.

### 2. 리눅스 기초 커맨드 강의
linux command(기초)
```bash
man     # 명령어 매뉴얼 확인
cd      # 디렉터리 이동
pwd     # 현재 경로 출력
ls      # 디렉터리 목록 출력
touch   # 파일 생성
mkdir   # 디렉터리 생성
```

ros2 command(기초)
```bash
ros2 node list                           # 실행 중인 노드 목록 확인
ros2 topic list                          # 퍼블리시 중인 토픽 목록 확인
ros2 topic echo <topic_name>             # 토픽 데이터 확인
colcon build --symlink-install --packages-select <package_name>       # 선택 패키지 빌드

```

bashrc에 자주 사용하는 source 및 명령어 alias 걸기 실습
```bash
cd ~; code ./.bashrc
source /opt/ros/humble/setup.bash
alias 'c'=clear
alias 'e'=exit
alias 'cb'=colcon build --symlink-install
```

### 3. roboflow, github 계정 생성 및 간단한 사용방법 제시
https://roboflow.com/

https://github.com/

### 4. 팀별로 
https://www.youtube.com/watch?v=DgEK8iT9QeA&list=PLIyoAG_PPqRfgHJl1UnMCg3h4-mE_eBfF&index=19


[![Simulation Video](http://img.youtube.com/vi/DgEK8iT9QeA&list=PLIyoAG_PPqRfgHJl1UnMCg3h4-mE_eBfF&index=19/0.jpg)](https://www.youtube.com/watch?v=DgEK8iT9QeA&list=PLIyoAG_PPqRfgHJl1UnMCg3h4-mE_eBfF&index=19)

### 5. 초단기 자율주행 차량 해커톤


