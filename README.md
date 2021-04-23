# moveit_test

ourarm folder = single arm manipulator cad file 이용해서 만든 urdf파일

final_config = 위의 urdf 파일을 이용해서 moveit 의 setup assistant 이용해서 생성된 폴더

final_gazebo = https://github.com/eYSIP-2017/eYSIP-2017_Robotic_Arm/wiki/Interfacing-MoveIt%21-with-Gazebo
위의 링크에서 참조할 때. git에서 다운받은 폴더. (moveit setup assistant를 통해 자동으로 생성되지는 않음)
이 폴더안에 config/launch 파일들을 우리 manipulator에 맞게 수정해서 사용했었음.

move_robot = c++/c/python 이용해서 메니퓰레이터 제어하는 코드들 모음폴더

position_estimation_test = bag files for single arm & mobile test

*** single arm 제어에만 이용함 ****
*** ourarm / joint 1~7 과 같은 이름들 ***

dualarm folder = 이 폴더에 있는 urdf 이용해서 moveit에서만 시뮬레이션으로 구현했었음.
실제로 dual-arm 제어는 한 적이 없으므로 dual-arm ver을 gazebo와 연동시켰던 적은 없음.
새로 구현해야 함




무빗과 가제보 연동 했을 때 무빗에서는 플래닝 되는데 가제보에서 execute 안 되던 이유 해결방법
(Unable to identify any set of controllers that can actuate the specified joints 라는 에러가 발생함)
: ros_controllers.yaml 의 controller list 가 []으로 남아있었기 때문인듯.
참고 링크
https://answers.ros.org/question/320990/moveit-unable-to-identify-any-set-of-controllers/


