![header](https://capsule-render.vercel.app/api?type=wave&color=auto&height=300&section=header&text=자율주행&nbsp;최단경로&nbsp;추종&nbsp;차량&fontSize=50)   
  [![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2Fjuniel1299%2FCapstoneDesign%2Fhit-counter&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)](https://hits.seeyoufarm.com)



<h1>💻 Capstone Design(졸업 작품) <br>
주제 : Shortest Path Following Vehicle (자율주행 최단경로 추종 차량)
</h1>

  <h2> 프로젝트 기간 : 2023년 3월 15일 ~ 2023년 11월 29일 (총 260일) <br>
    프로젝트 총 인원 : 4명
</h2>

<h2>
  📝 요약
</h2>
  천장에 설치된 카메라를 통해 맵 전체를 촬영 <br>
  -> 화면 전체를 numPy를 활용하여 좌표값을 부여 , 색을 통해 장애물, 현재 차량의 위치와 기울어짐을 계산  <br>
  -> 촬영 화면을 클릭하여 출발점과 도착점을 지정 <br>
  -> 출발점과 도착점의 최단 경로를 계산 후 해당 경로를 따라서 차량이 이동하도록 설계하였습니다. 

  ---
  
<h2> :pushpin: 상세기능 </h2>
1. opencv 라이브러리를 통해 파란색, 초록색, 노란색을 인식합니다. <br>
2. Python 언어 특성상 포인터가 존재하지 않으므로 numPy 라이브러리를 통해 카메라를 통해 촬영되는 화면에 좌표값을 지정합니다. <br>
3. 차량에 부착되어 있는 파란색을 인식하여 차량의 현재 위치를 알 수 있습니다. <br>
4. 차량 앞 부분에 있는 초록색과 차량 뒷 부분에 있는 파란색을 인식하여 두 색에 대한 각도를 통해 현재 차량이 얼마나 기울어져 있는가를 계산하였습니다. <br>
5. 최단경로의 경우 A* 알고리즘을 활용하여 최단경로를 출력하였습니다. <br>

--- 


<h2> 📕 Skill </h2>

![Arduino](https://img.shields.io/badge/-Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)
![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![NumPy](https://img.shields.io/badge/numpy-%23013243.svg?style=for-the-badge&logo=numpy&logoColor=white)
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)

---

<h2> :telescope: IDE </h2>

![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white)
![Octave](https://img.shields.io/badge/OCTAVE-darkblue?style=for-the-badge&logo=octave&logoColor=fcd683)

---
<h2> 🗄 Document </h2>

<div><a src="https://github.com/juniel1299/CapstoneDesign/blob/main/%EC%BA%A1%EC%8A%A4%ED%86%A42_3%EC%B0%A8%EB%B3%B4%EA%B3%A0%EC%84%9C_%EC%9E%A5%EC%9B%90%EC%A4%80_60195101_7%EC%A1%B0.pdf"> 최종 개인 보고서 </a></div>
<div><a src="https://github.com/juniel1299/CapstoneDesign/blob/main/%EC%88%98%EC%83%81_inside.jpg"> 수상 내역 </a></div>
<div><a src="https://github.com/juniel1299/CapstoneDesign/blob/main/%EC%BA%A1%EC%8A%A4%ED%86%A4%EB%94%94%EC%9E%90%EC%9D%B8_%EC%B5%9C%EC%A2%85%EB%B3%B4%EA%B3%A0%EC%84%9CPPT.pdf"> 최종 PPT </a>  </div>



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

기존에 사용하려던 라이다 센서를 제거하는 대신 맵 중앙 위치의 천장에 휴대폰 거치대를 설치 -> 휴대폰 거치대에 휴대폰을 놓은 뒤 맵 전체를 카메라를 통해 촬영 -> ip webcam 을 통하여 같은 와이파이 안에 접속한 휴대폰의 촬영 화면을 컴퓨터와 공유 -> 영상처리 코드와 실시간 촬영을 연결하기 위해서 VSCODE IDE에  화면 캡쳐 주소를 ip webcam에서 제공하는 주소로 설정하여 VSCODE IDE에 작성한 코드 실행시 휴대폰 카메라가 촬영하는 화면을 영상처리 할 수 있도록 가져온다.    

![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white) 사용 언어를 ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)으로 하였는데 ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)은 언어 자체에 포인터가 없기 때문에 ![NumPy](https://img.shields.io/badge/numpy-%23013243.svg?style=for-the-badge&logo=numpy&logoColor=white) 라이브러리를 통하여 눈에 보이지 않는 가상의 행렬을 화면에 깔아 임의적으로 xy좌표를 생성해 현재 차량 위치에 대한 좌표, 출발점 , 도착점 등 다양한 좌표값을 생성해낼 수 있음 .  

차량의 현재 위치 , 차량의 기울어짐(각도) , 장애물들을 색깔을 이용하여 카메라가 인식을 하여 데이터를 얻어내고 그 데이터를 알고리즘에서 사용함 -> A* 알고리즘의 경우 차량의 현재 위치 , 장애물에 대한 데이터를 이용하며 Pure Pursuit의 경우 차량의 기울어짐(각도). 현재 위치를 이용해 차량이 어떻게 기울어지지 않고 이동할 수 있는지 계산할 때 이용됨 .     

교내 전기공학과 캡스톤 경진대회 수상하였음 .  

수상 파일은 리포지토리 안에 존재. (**[수상_front.jpg](https://github.com/juniel1299/CapstoneDesign/blob/main/%EC%88%98%EC%83%81_front.jpg)**,**[수상_inside.jpg](https://github.com/juniel1299/CapstoneDesign/blob/main/%EC%88%98%EC%83%81_inside.jpg)**)  

동작은 안에 동영상 파일을 통해 확인. (**[Play.mp4](https://github.com/juniel1299/CapstoneDesign/blob/main/Play.mp4)**)  

자세한 내용은 해당 리포지토리에 첨부한 ppt 자료를 통해 확인하시길 바랍니다. (**[캡스톤디자인_최종보고서PPT.pdf](https://github.com/juniel1299/CapstoneDesign/blob/main/%EC%BA%A1%EC%8A%A4%ED%86%A4%EB%94%94%EC%9E%90%EC%9D%B8_%EC%B5%9C%EC%A2%85%EB%B3%B4%EA%B3%A0%EC%84%9CPPT.pdf)**) 

## 시연 영상



https://github.com/juniel1299/CapstoneDesign/assets/62318700/8ac97ec9-4746-4a70-bc46-1ea80fd3dec5


---

# 파일 정리
아두이노파일과 pure pursuit 알고리즘의 경우 보드에 컴파일하고 모터가 실제로 조절이 되어야 하므로 따로 파일을 만들어 아두이노에 컴파일 

컴퓨터에서 실행되는 파일은 code_file5 로 안에는 opencv와 A* 알고리즘이 존재함 . 


---

[해당 프로젝트 진행 관련 블로그 글](https://problem-child.tistory.com/84)

