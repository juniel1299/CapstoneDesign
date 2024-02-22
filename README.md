![header](https://capsule-render.vercel.app/api?type=wave&color=auto&height=300&section=header&text=자율주행&nbsp;최단경로&nbsp;추종&nbsp;차량&fontSize=50)   
  [![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2Fjuniel1299%2FCapstoneDesign%2Fhit-counter&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)](https://hits.seeyoufarm.com)
# Capstone Design 
# 주제 : Shortest path following vehicle (자율주행 최단경로 추종 차량)
## 개발환경
 window + Linux(Raspberry PI , Ubuntu)   [ 최종적으로는 window만 이용하였습니다. 이유는 아래 참고 ]
## LANGUAGE
python , C (Arduino)
## lib
numpy , opencv(cv2) , time , math , pyserial , etc...
## HW 
lidar ( YDLIDAR X2 ) , Raspberry PI 4B , L298N Motor Driver , Arduino Uno , DC Motor (No encoder) , Wheel , SmartPhone Camera [ 최종적으로 Raspberry Pi , lidar 센서는 이용하지 않았습니다. 이유는 아래 참고 ]
## Algorithm
A* Algorithm , Pure persuit Algorithm



![py](https://img.shields.io/badge/Python-14354C?style=for-the-badge&logo=python&logoColor=white) ![c](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)  <br/>
![window](https://img.shields.io/badge/Windows-0078D6?style=for-the-badge&logo=windows&logoColor=white)   <br/>
![arduino](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)<br/>
![vscode](https://img.shields.io/badge/Visual_Studio_Code-0078D4?style=for-the-badge&logo=visual%20studio%20code&logoColor=white)<br/>

---

최종에서 라즈베리파이와 라이다 센서에 대한 사용을 하지 않는 것으로 수정

이유 1 ) 라즈베리파이에서 수행할 수 있는 그래픽 처리 한계로 인하여 opencv 인식 속도가 너무 느림 -> 코드 동작 화면에 버퍼링이 생김에 따라 차량이 정상동작 할 수 없음.

이유 2) 라이다 센서에 대한 개념 이해 부족 , 그래픽 처리 한계로 인하여 벽 스캔 후 그래픽 생성 속도가 너무 느림 -> 코드 동작 화면에 버퍼링이 생김에 따라 차량이 정상동작 할 수 없음.

# 사용한 라이브러리에 대한 이유
## numpy
 python 언어 특성상 포인터가 존재하지 않기 때문에 numpy 라이브러리를 이용해 눈에 보이지 않는 가상의 행렬을 생성하여 포인터를 대체하여 해당 위치의 x좌표와 y좌표를 생성하는 용도로 이용함   
## opencv(cv2) 
맵 전체를 카메라를 통해 받아와 차량의 현재위치, 차량의 기울어짐(각도), 벽과 장애물에 인식을 하기 위해서 사용 
## pyserial 
아두이노와 컴퓨터 사이 시리얼 모니터 통신을 할 수 있도록 하기 위하여 사용
## time 
연산하는데 걸리는 시간을 측정하기 위해 이용하였음. 
## math  
opencv 앞에서 나온 각도값과 알고리즘에서 사용하는 계산 값을 구하기 위해 사용

# 해당 프로젝트에서의 본인의 역할

기존에 사용하려던 라이다 센서를 제거하는 대신 맵 중앙 위치의 천장에 휴대폰 거치대를 설치 -> 휴대폰 거치대에 휴대폰을 놓은 뒤 맵 전체를 카메라를 통해 촬영 -> ip webcam 을 통하여 같은 와이파이 안에 접속한 휴대폰의 촬영 화면을 컴퓨터와 공유 -> vscode 화면 캡쳐 주소를 ip webcam 주소로 설정하여 코드 실행시 휴대폰 카메라가 촬영하는 화면을 그대로 가져옴   

![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white) 사용 언어를 ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)으로 하였는데 ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)은 언어 자체에 포인터가 없기 때문에 ![NumPy](https://img.shields.io/badge/numpy-%23013243.svg?style=for-the-badge&logo=numpy&logoColor=white) 라이브러리를 통하여 눈에 보이지 않는 가상의 행렬을 화면에 깔아 임의적으로 xy좌표를 생성해 현재 차량 위치에 대한 좌표, 출발점 , 도착점 등 다양한 좌표값을 생성해낼 수 있음 .  

차량의 현재 위치 , 차량의 기울어짐(각도) , 장애물들을 색깔을 이용하여 카메라가 인식을 하여 데이터를 얻어내고 그 데이터를 알고리즘에서 사용함 -> A* 알고리즘의 경우 차량의 현재 위치 , 장애물에 대한 데이터를 이용하며 Pure Pursuit의 경우 차량의 기울어짐(각도). 현재 위치를 이용해 차량이 어떻게 기울어지지 않고 이동할 수 있는지 계산할 때 이용됨 .   

교내 전기공학과 캡스톤 경진대회 수상하였음 .

수상 파일은 리포지토리 안에 존재. 

동작은 안에 동영상 파일을 통해 확인.

자세한 내용은 해당 리포지토리에 첨부한 ppt 자료를 통해 확인하시길 바랍니다.


---

# 파일 정리
아두이노파일과 pure pursuit 알고리즘의 경우 보드에 컴파일하고 모터가 실제로 조절이 되어야 하므로 따로 파일을 만들어 아두이노에 컴파일 

컴퓨터에서 실행되는 파일은 code_file5 로 안에는 opencv와 A* 알고리즘이 존재함 . 



