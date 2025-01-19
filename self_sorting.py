# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 200, 200

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            get_current_posx,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            mwait,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            trans,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    JReady = posj([0, 0, 90, 0, 90, 0])
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
   # 계산된 좌표 값
    x00 = [498.748, 97.436, 23.648, 17.387, 179.305, 15.346]
    x10 = [498.748, 44.982, 23.648, 17.387, 179.305, 15.346]
    x20 = [498.748, -7.472, 23.648, 17.387, 179.305, 15.346]
    x01 = [548.415, 97.436, 23.648, 17.387, 179.305, 15.346]
    x11 = [548.415, 44.982, 23.648, 17.387, 179.305, 15.346]
    x21 = [548.415, -7.472, 23.648, 17.387, 179.305, 15.346]
    x02 = [598.082, 97.436, 23.648, 17.387, 179.305, 15.346]
    x12 = [598.082, 44.982, 23.648, 17.387, 179.305, 15.346]
    x22 = [598.082, -7.472, 23.648, 17.387, 179.305, 15.346]
    
    k11 = [548.415, -103.545, 23.648, 17.387, 179.305, 15.346]


    global pallet_i
    pallet = [0,0,0,0,2,0,0,0,0]
    top_pallet = [x00, x01, x02, x10, x11, x12, x20, x21, x22]


    def wait_digital_input(sig_num):
        # while not get_digital_input(sig_num):
        #     wait(0.5)
        #     print("Wait for digital input", sig_num)
        #     pass
        wait(1)
        
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        print("Release", end='')
        wait_digital_input(2)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        # if flag:
        #     print("Grip", end='')
        wait_digital_input(1)
        wait(1.0)

    def sort_block(z):
        if z < 50: 
            if not 0 in pallet[0:3]:
                return "s", "blank"
            index = pallet.index(0,0,3)
            print("S")
            return "s",index
        elif 50 <= z and z < 60: 
            if not 0 in pallet[3:6]:
                return "m", "blank"
            index = pallet.index(0,3,6)
            print("M")
            return "m", index
        elif 60 <= z:
            if not 0 in pallet[6:9]:
                return "l", "blank"
            index = pallet.index(0,6,9)
            print("L")
            return "l", index
        else:
            print("Error: height")


    def check(base_pose):
        base_point_up =  trans(base_pose, [0,0,100,0,0,0], DR_BASE, DR_BASE )
        force_start_point =  trans(base_pose, [0,0,47,0,0,0], DR_BASE, DR_BASE)
        movel(base_point_up, vel=VELOCITY, acc=ACC)
        movel(force_start_point, vel=VELOCITY, acc=ACC)#내려감
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass
        release_compliance_ctrl()
        time.sleep(2.0)
        # z축 sorting
        place = get_current_posx()
        result = sort_block(place[0][2])
        movel(base_point_up, vel=VELOCITY, acc=ACC)
        return result

    def pick(base_pose):
        base_point_up =  trans(base_pose, [0,0,100,0,0,0], DR_BASE, DR_BASE )
        movel(base_point_up, vel=VELOCITY, acc=ACC)
        release()
        movel(base_pose, vel=VELOCITY, acc=ACC)
        grip()
        movel(base_point_up, vel=VELOCITY, acc=ACC)
        print("current position4 : ", get_current_posx())

    def place(base_pose):
        base_pose_up = trans(base_pose, [0,0,70,0,0,0], DR_BASE, DR_BASE)
        force_start_point = trans(base_pose, [0,0,27,0,0,0], DR_BASE, DR_BASE)
        movel(base_pose_up, vel=VELOCITY, acc=ACC)
        print("current position2 : ", get_current_posx())
        movel(force_start_point, vel=VELOCITY, acc=ACC)#내려감
        print("current position3 : ", get_current_posx())
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], time=0.1, mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass
        release_compliance_ctrl()
        time.sleep(0.5)
        release()
        movel(base_pose_up, vel=VELOCITY, acc=ACC)
        grip()

    def handle_block_mismatch(block_size, assigned_index):
        
        if assigned_index == "blank":
            start = pallet.index(0)
            end = pallet.index(2)
            pick(top_pallet[start])
            place(top_pallet[end]) 
            pallet[start] = 2
            pallet[end] = 1
            return

        block_size2, _ = check(top_pallet[assigned_index])
        start = pallet.index(0)
        end = assigned_index
        buffer = pallet.index(2)
        if block_size2 == block_size:
            pallet[assigned_index] = 1
            print(pallet)
            if pallet[assigned_index+1] == 2:
                handle_block_mismatch(block_size, assigned_index+2)
                return
            handle_block_mismatch(block_size, assigned_index+1)
        else:
            pick(top_pallet[end])
            place(top_pallet[buffer])  # 미리 지정된 임시 위치
            pick(top_pallet[start])
            place(top_pallet[end])
            print(pallet)
            pallet[buffer] = 0
            print(pallet)
            pallet[start] = 2
            print(pallet)
            pallet[end] = 1
            print(pallet)

    
    if rclpy.ok():
        movej(JReady, vel=VELOCITY, acc=ACC)
        release()
        movel(trans(x11, [0,0,100,0,0,0], DR_BASE, DR_BASE ), vel=VELOCITY, acc=ACC)
        movel(x11, vel=VELOCITY, acc=ACC)
        grip()
        movel(trans(x11, [0,0,100,0,0,0], DR_BASE, DR_BASE ), vel=VELOCITY, acc=ACC)
        movel(trans(k11, [0,0,100,0,0,0], DR_BASE, DR_BASE ), vel=VELOCITY, acc=ACC)
        movel(k11, vel=VELOCITY, acc=ACC)
        release()
        movel(trans(k11, [0,0,100,0,0,0], DR_BASE, DR_BASE ), vel=VELOCITY, acc=ACC)
        
        grip()
        while 0 in pallet:
            print(pallet)
            block_size, assigned_index = check(top_pallet[pallet.index(0)])
            if pallet.index(0) < 3:
                if block_size == "s":   pallet[pallet.index(0)] = 1
                else:
                    handle_block_mismatch(block_size, assigned_index)

            elif pallet.index(0) >=3 and pallet.index(0) <6:
                if block_size == "m":   pallet[pallet.index(0)] = 1
                else: 
                    handle_block_mismatch(block_size, assigned_index)
            else:
                if block_size == "l":   pallet[pallet.index(0)] = 1
                else:
                    handle_block_mismatch(block_size, assigned_index)

        release()
        movel(trans(k11, [0,0,100,0,0,0], DR_BASE, DR_BASE ), vel=VELOCITY, acc=ACC)
        movel(k11, vel=VELOCITY, acc=ACC)
        grip()
        movel(trans(k11, [0,0,100,0,0,0], DR_BASE, DR_BASE ), vel=VELOCITY, acc=ACC)
        
        target = top_pallet[pallet.index(2)]

        movel(trans(target, [0,0,100,0,0,0], DR_BASE, DR_BASE ), vel=VELOCITY, acc=ACC)
        movel(target, vel=VELOCITY, acc=ACC)
        release()
        movel(trans(target, [0,0,100,0,0,0], DR_BASE, DR_BASE ), vel=VELOCITY, acc=ACC)
        


        
        

    rclpy.shutdown()


if __name__ == "__main__":
    main()