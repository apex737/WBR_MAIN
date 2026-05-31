Q1. 핫스팟에 esp와 PC가 둘다 붙어있는 경우에만 csv 받을수 있는 이유?
A1. 핫스팟은 사설 IP (192.168.x.x) 를 단말에 할당하는데, 외부 라우터가 이 주소를 알 수 없기 때문.


------------------ 트러블 슈팅 ------------------
LQR:
1. velocity 부호 반전 수정
2. LQR theta 게인 키워서 수렴 범위 확대
- Step 1: LQR theta 게인 키우기
- Step 2: EKF Q_cov 조정

그 다음:
3. delay-augmented LQR으로 비최소위상 보상



