# 🚀  Autonomous Navigation Project

## 🔹 프로젝트 개요
이 프로젝트는 **Stanley Controller 및 A* 알고리즘을 활용한 자율 주행 로봇 제어 및 경로 계획 시스템**입니다.  
장애물 맵을 기반으로 A* 알고리즘을 사용하여 최적의 경로를 생성하고,  
Stanley Controller를 이용하여 로봇이 생성된 경로를 따라 이동할 수 있도록 합니다.

## 🚀 주요 기능
✅ **A* 알고리즘 기반 최적 경로 생성**  
✅ **Stanley Controller를 통한 차량 조향 제어**  
✅ **경로 보간 및 곡률 기반 보정 기능 포함**  
✅ **장애물 맵을 활용한 안전한 경로 탐색**  

---
### 📖 내용

- 정해진 map과 robot 내에서 경로를 찾고 그 경로를 따라가는 자율 주행 시뮬레이션을 구성
- 경로를 찾는 것은 A*알고리즘을 사용
- 경로를 따라가는 것은 stanley-controller를 사용
- shell-script를 활용한 시뮬레이션 자동화

![스크린샷 2024-12-17 003235.png](attachment:8a1a9601-8ae0-4bd9-b4e0-158ec999721c:스크린샷_2024-12-17_003235.png)

![스크린샷 2024-12-18 181522.png](attachment:0ee91ed3-7a58-4ab7-bacb-618119a01208:c85c5bc3-84fe-4c89-a82a-841743db0bc6.png)

### 🙋‍♂️ 역할

- 환경 구성 (맵, 로봇)
- path-planning, path-tracking
- 튜닝을 통한 시간 단축
## 🛠️ 사용 기술
- **언어**: Python (NumPy, SciPy)  
- **경로 계획**: A* 알고리즘 + 곡률 기반 경로 최적화  
- **제어 시스템**: Stanley Controller (PID 제어 포함)  
- **시뮬레이션**: Python 기반 차량 모델  

---


