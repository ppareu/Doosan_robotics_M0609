import rclpy
import DR_init
import math

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_BASE,
            set_digital_output,
            get_digital_input,
            wait,
            trans,
            ON,
            OFF,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            get_current_posx,
            
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():

        # 컵 두개 잡는 좌표
        two_cups_posx = posx(409.16, -76.96, 187.50, 167.83, -177.88, 168.34)
        # 2개 저장하는 좌표 
        store_cup_pos = posx(409.16, 79.10, 170.00, 167.83, -177.88, 168.34)
        dis_posx, dis_posy = two_cups_posx[0] - store_cup_pos[0], two_cups_posx[1] - store_cup_pos[1]   

        # start stack
        start_stack_pos = posx(409.16, -76.96, 200.00, 167.83, -177.88, 168.34)
        
        # 컵 3개 잡는 좌표 
        three_cups_posx = posx(409.16, -76.96, 90.00, 167.83, -177.88, 168.34)

        #마지막
        last_pos1 = [posj(43.13, 49.83, 79.11, 122.98, -124.09, -130.59), #시작
        #<---release
                     posj(42.99, 55.22, 79.56, 125.51, -120.92, -125.30),  #첫번째 grip
        #<---grip             
                     posj(43.46, 43.62, 79.16, 119.48, -127.52, -136.86), #뒤집기 시작 위치
                     posj(43.46, 43.62, 79.16, 119.48, -127.52, -316.91), #뒤집기
                     posj(42.94, 55.71, 79.92, 126.04, -120.06, -304.29), #뒤집기 완료후 release
        #<---release
                     posj(42.60, 56.93, 79.31, 126.01, -119.52, -303.87),
        #<---grip
                     posj(42.44, 39.68, 74.02, 111.94, -130.55, -328.02),
                     posj(27.28, 35.54, 83.79, 104.56, -114.30, -327.46)]


        diff_w = 77.86
        diff_h = diff_w * (math.sqrt(3)/2)
        diff_h2 = diff_h * (2/3) - 5
        height = 0
        cup_h = 40

        stack_cup_list = []

        def stack_cup(len_of_tri, stack_start):
            if len_of_tri == 0:
                return stack_start
        #     else:
                # if not is_first:
                #     movel(list(trans(pre_stack_start, [0, 0, 10, 0, 0, 0], ref=DR_BASE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)                        
                #     movel(three_cups_posx, vel=VELOCITY, acc=ACC, ref=DR_BASE)
                #     grip()
                #     movel(list(trans(pre_stack_start, [0, 0, 10, 0, 0, 0], ref=DR_BASE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
                #     movel(list(trans(stack_start, [0, 0, 10, 0, 0, 0], ref=DR_BASE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)   
                #     movel(list(trans(stack_start, [0, 0, -50, 0, 0, 0], ref=DR_BASE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
                #     ungrip()
                
            for i in range(len_of_tri): 
                
                stack_start_l = list(trans(stack_start, [diff_h * i, -diff_w * i / 2, 0, 0, 0, 0], ref=DR_BASE))

                
                for j in range(i + 1): 
                    stack = list(trans(stack_start_l, [0, diff_w * j, 0, 0, 0, 0], ref=DR_BASE))
                    stack_cup_list.append(stack)
                # check_object_pos(stack_start) 
                # move_and_release2(stack)

            stack_cup(len_of_tri-1, list(trans(stack_start, [diff_h2, 0, cup_h, 0, 0, 0], ref=DR_BASE)))

           


        def check_object_pos(stack_point):
            check_compliance_grip()
            movel(stack_point, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass

            # 여기서 부터는 외력이 느껴질때 get_pose사용해서 z축 높이 확인
        
            object_pos, _ = get_current_posx()
            print(object_pos[2])
            release_compliance_ctrl()

            grip_point = list(trans(object_pos, [0, 0, 10, 0, 0, 0], DR_BASE, DR_BASE))
            movel(grip_point, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            go_down_and_grip_stackcup(grip_point)
            return object_pos
            


        def ungrip():
            set_digital_output(2, ON)
            set_digital_output(1, OFF)
            wait_digital_input(2)

        def grip():
            set_digital_output(1, ON)
            set_digital_output(2, OFF)
            wait_digital_input(1)

        def check_compliance_grip():
            set_digital_output(1, ON)
            set_digital_output(2, ON)
    
        # 컵 두개만 집을 때 사용 
        def go_down_and_grip(origin):
            ungrip()
            grip_pos = two_cups_posx
            movel(grip_pos, v=100, a=100)
            grip()
            movel(origin, v=100, a=100)

        def go_down_and_grip_stackcup(origin):
            ungrip()
            grip_pos = list(trans(origin, [0, 0, -25, 0, 0, 0], DR_BASE, DR_BASE))
            movel(grip_pos, v=100, a=100)
            grip()
            up_grip_pos = list(trans(origin, [0, 0, 100, 0, 0, 0], DR_BASE, DR_BASE))
            movel(up_grip_pos, v=100, a=100)
            
        def move_and_release(origin, direction=False):
            if direction:
                is_reverse_pos = dis_posy
            else:
                is_reverse_pos = -dis_posy
                
            temp_pos = list(trans(origin, [0, is_reverse_pos, 0, 0, 0, 0], DR_BASE, DR_BASE))
            movel(temp_pos, v=100, a=100)
            ungrip_pos = list(trans(origin, [0, is_reverse_pos, -200, 0, 0, 0], DR_BASE, DR_BASE))
            movel(ungrip_pos, v=80, a=60)
            ungrip()
            movel(temp_pos, v=100, a=100)

        def move_and_release2(goal):
            temp_pos = list(trans(goal, [0, 0, 50, 0, 0, 0], DR_BASE, DR_BASE))
            movel(temp_pos, v=100, a=100)
            ungrip_pos = list(trans(goal, [0, 0, -115, 0, 0, 0], DR_BASE, DR_BASE))
            movel(ungrip_pos, v=80, a=60)
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
            ungrip()
            movel(temp_pos, v=100, a=100)

        def move_and_release3(goal):
            movel(list(trans(three_cups_posx, [0, 0, 70, 0, 0, 0], ref=DR_BASE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)                        
            movel(three_cups_posx, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            grip()
            movel(list(trans(three_cups_posx, [0, 0, 70, 0, 0, 0], ref=DR_BASE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            movel(list(trans(goal, [0, 0, 10, 0, 0, 0], ref=DR_BASE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)   
            movel(list(trans(goal, [0, 0, -123, 0, 0, 0], ref=DR_BASE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
            ungrip()
            movel(list(trans(goal, [0, 0, 10, 0, 0, 0], ref=DR_BASE)), vel=VELOCITY, acc=ACC, ref=DR_BASE)
        



        ############################################### 실행 
        
        #시작 위치 
        startp = posj(0, 0, 90, 0, 90, 0)
        movej(startp, v=80, a=60)
        ungrip()
        # 컵 두개 먼저 이동 
        move_two_cups = list(trans(two_cups_posx, [0, 0, 100, 0, 0, 0], DR_BASE, DR_BASE))
        movel(move_two_cups, v=100, a=100)  
        go_down_and_grip(two_cups_posx)
        movel(move_two_cups, v=100, a=100)
        move_and_release(move_two_cups)

        # stack 좌표 생성 
        stack_cup(3, start_stack_pos)
        print(stack_cup_list)

        # 컵 쌓기 시작 3 = 삼각형 빗변이 컵 3개
        for i in range(len(stack_cup_list)):    
            if i == 0: 
                start_point = stack_cup_list[0]
                object_pos = start_point
            elif i == 6:
                move_and_release3(stack_cup_list[i])
                start_point = stack_cup_list[i]
                object_pos = start_point 
                #object_pos[2] -= 100
            elif i == 9:
                # move_final_cups = list(trans(two_cups_posx, [0, dis_posy, 100, 0, 0, 0], DR_BASE, DR_BASE))
                # movel(move_final_cups, v=100, a=100)  
                # go_down_and_grip(move_final_cups)
                # movel(move_final_cups, v=100, a=100)
                # movel(list(trans(stack_cup_list[i], [0, 0, 50, 0, 0, 0, 0], ref=DR_BASE)), v=100, a=100)
                # movel(stack_cup_list[i], v=100, a=100)
               
                for i in range(len(last_pos1)):
                        
                    movej(last_pos1[i],vel=VELOCITY, acc=ACC)
                    if i == 0 or i == 4 or i == 7:
                        ungrip()
                    if i == 1 or i == 5:
                        grip()
                
            else:
                object_pos = check_object_pos(object_pos)
                move_and_release2(stack_cup_list[i])

        grip()
        ungrip()
        
        last_pos, _ = get_current_posx(ref=DR_BASE)
        movel(list(trans(last_pos, [0, 70, 0, 0, 0, 0], ref=DR_BASE)),vel=VELOCITY, acc=ACC)
        movej(startp, v=80, a=60)
        break
        

    rclpy.shutdown()

if __name__ == "__main__":
    main()

