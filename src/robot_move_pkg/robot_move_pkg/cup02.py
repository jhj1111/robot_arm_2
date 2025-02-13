import rclpy
import math
# Doosan 로봇 설정 (단일 로봇 기준)
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("cup", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        # Doosan Robot API 임포트
        from DSR_ROBOT2 import (
            release_compliance_ctrl, check_force_condition, task_compliance_ctrl,
            set_desired_force, set_tool, set_tcp, movej, movel, movesx, movejx,
            set_velx, set_accx, set_velj, set_accj, DR_FC_MOD_REL, DR_MV_MOD_REL,
            DR_MV_MOD_ABS, DR_AXIS_Z, DR_BASE, get_external_torque,DR_MV_RA_DUPLICATE, DR_MV_RA_OVERRIDE,
            get_current_solution_space, drl_script_stop, DR_QSTOP, DR_TOOL,
            DR_WORLD, posx, trans, set_digital_output, get_digital_input,
            wait, get_current_posx, get_current_posj, amovel,  DR_MVS_VEL_NONE ,DR_MVS_VEL_CONST
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    # 툴과 TCP 설정
    set_tool("Tool Weight_2FG")  # 툴의 무게 설정
    set_tcp("2FG_TCP")  # TCP (Tool Center Point) 설정
    # 변수 초기화 (컵의 크기 및 개수 설정)
    global H_, H_top_, num, d_, D_, i_num, start
    start = []
    i_num = 9  # 현재 쌓을 컵의 인덱스
    num = 12  # 총 컵 개수
    d_ = 55  # 컵의 작은 원 지름
    D_ = 76  # 컵의 큰 원 지름
    H_ = 95  # 컵의 높이
    H_top_ = 61.00 + H_ + num * 10  # 초기 높이 설정
    
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass
    def setvel(v,a):
        set_velx(v)
        set_accx(a)
    def setvelj(v, a):
        set_velj(v)
        set_accj(a)
    def grab_signal():
        #wait_digital_input(1, 1)
        #wait_digital_input(2, 0)
        wait_digital_input(1)
    def open_signal():
        #wait_digital_input(1, 0)
        #wait_digital_input(2, 1)
        wait_digital_input(2)
    def close_signal():
        #wait_digital_input(1, 1)
        #wait_digital_input(2, 1)
        wait_digital_input(1)
        wait_digital_input(2)
    def joint_open():
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        open_signal()
    def joint_close():
        set_digital_output(1, 1)
        set_digital_output(2, 1)
        close_signal()
    def joint_grab():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        grab_signal()
    
    # 힘 감지를 이용한 수직 이동 함수
    def vertical_move_with_force_detection():
        task_compliance_ctrl(stx=[700, 700, 300, 300, 300, 300])  # 힘 제어 활성화
        set_desired_force(fd=[0, 0, -25, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):  # 특정 힘이 감지될 때까지 대기
            pass
        return get_current_posx()
    
    
    # 삼각형 배열의 포인트 생성 함수
    def point_line(x, y, h, n):
        xi = x
        points = []
        for i in range(n + 1):
            if i != 0:
                y += D_ * math.sqrt(3) / 2
            for j in range(n - i):
                if j != 0:
                    x += 80
                points.append((x, y, h))
            x = xi + 40 * (i + 1)
        return points
    # 모든 컵의 위치를 계산하는 함수
    def points(x, y, h, n):
        cup_points = []
        for i in range(n):
            cup_points += point_line(x + i * 40, y + i * (D_ * math.sqrt(3) / 6), h + i * H_, n - i)
        return cup_points
    # 물체를 집는 동작 수행 함수
    def pick_obj(x, y):
        global H_top_, start
        joint_close()
        (_, _, _, rx, ry, rz), sol = get_current_posx()
        movejx(posx(x, y, H_top_, rx, ry, rz), ra= DR_MV_RA_OVERRIDE, r= 150 , sol=sol)
        new_pos, _ = vertical_move_with_force_detection()
        start = new_pos[:2]
        H_top_ = new_pos[2]
        release_compliance_ctrl()
        movejx(posx([0, 0, 5, 0, 0, 0]), mod=DR_MV_MOD_REL,  sol=sol)
        joint_open()
        movejx(posx([0, 0, -20, 0, 0, 0]), mod=DR_MV_MOD_REL,  sol=sol)
        joint_grab()
    # 물체를 목표 위치에 놓는 함수
    def put_obj(x, y, z):
        global i_num
        (_, _, zi, rx, ry, rz), sol = get_current_posx()
        zi = max(zi, 88.45 + z + 100)
        # movejx(posx(x, y, zi, rx, ry, rz), r= 150, sol=sol)
        # movejx(posx(x, y, 88.45 + z + 100, rx, ry, rz), r= 150, sol=sol)
        x1 = posx(x, y, zi, rx, ry, rz)
        x2 = posx(x, y, 88.45 + z + 100, rx, ry, rz)
        # x3 = posx(x, y, 88.45 + z , rx, ry, rz)
        
        xlist = [x1,x2]
        movesx(xlist, vel=[60, 30], acc=[60, 30], vel_opt=DR_MVS_VEL_CONST)
        movejx(posx([0, 0, -100, 0, 0, 0]), mod=DR_MV_MOD_REL, r= 150, v=30, a=30, sol=sol)
        wait(0.5)
        joint_open()
        movejx(posx([0, 0, 100, 0, 0, 0]), mod=DR_MV_MOD_REL,  r= 150, sol=sol)
        i_num += 1
    ### 메인 코드 ###
    initialx = [417.86, 210.83, 212.20, 12.62, 180, -180]  # 시작 위치
    goalx = [289.94, -217.14, 89.45, 61, 180, -105]  # 목표 위치
    goal_points = points(goalx[0], goalx[1], 0, 3)  # 목표 위치 포인트 계산
    start = initialx[:2]
    while rclpy.ok() and i_num <= len(goal_points):
        if i_num == 0:
            joint_open()
        setvel(20, 20)
        setvelj(20, 20)
        movej([0, 0, 90, 0, 90, 0], ra= DR_MV_RA_OVERRIDE, r=200)  # 초기 자세 설정
        pick_obj(*start)  # 컵을 집기
        put_obj(*goal_points[i_num])  # 목표 위치에 놓기
    rclpy.shutdown()
if __name__ == "__main__":
    main()