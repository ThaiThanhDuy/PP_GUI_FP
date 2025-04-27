


    def read_voltage(self, flag):
            if flag:
                print('start_ arduino -----')
    def bat_dau_dan_duong(self):
        if self.arduino_charing.batdau_sac:
            goals = [self.listWidget_ds.item(index).text() for index in range(self.listWidget_ds.count())]
            print('Goals: ', goals)
            goal_ids = []
            for goal in goals:
                if 'Phòng' in goal:
                    goal_ids.append(int(goal.split()[-1]))
                elif 'Home' in goal:
                    goal_ids.append(RobConf.HOME_ID)
            
            print('Goal ids: ', goal_ids)
            if goals:  # kiểm tra danh sách có phần tử nào ko
                first_item = goals[0]
                self.label_tt.setText(first_item)  # label có ô vuông
                self.current_item_index = 0
            #print(self.currentPositionID, RobConf.TruocDockSacID)
            if self.currentPositionID == RobConf.TruocDockSacID: # dang o dock sac -> qt: chay
                    #set pose tai diem 100

                    #----------------- BUOC 1: SET POSE TU SQL ----------------
                    self.ros2_handle.goal_publisher.set_goal_from_sql(100)

                    #----------------- BUOC 2: CHAY MU RA NGOAI ----------------
                    self.ros2_handle.cmd_vel_publisher.move_with_command('Up', 0.3, 0)
                    time.sleep(2)
                    self.ros2_handle.cmd_vel_publisher.move_with_command('Stop', 0, 0)
                    time.sleep(2)
                    #----------------- BUOC 3: GHI VI TRI HIEN TAI LEN FILE ODOM ----------------               
                    Uti.writePose2File(filePath = file_path_pose_data,x=self.ros2_handle.odom_listener.data_odom[0],
                                        y=self.ros2_handle.odom_listener.data_odom[1],
                                        z=self.ros2_handle.odom_listener.data_odom[2],
                                        w=self.ros2_handle.odom_listener.data_odom[3])


                    #----------------- BUOC 4: DI DEN VI TRI MONG MUON ----------------      
                # self.hien_thi_phan_tu_dau_tien()
            else:
                self.dinhvi_dauhanhtrinh = False
                self.hien_thi_phan_tu_dau_tien()
                    # chuyển sang nút xacnhan để sang vị trí bàn tiếp theo
            
                # --------------------- phat am thanh -------------------------
                Uti.RobotSpeakWithPath('voice_hmi_new/new_nhuong_duong.wav')
             
        else:
             Uti.RobotSpeakWithPath('voice_hmi_new/toidangdisac.wav')
             print('toi dang di sac')
    def hien_thi_phan_tu_dau_tien(self):
        ##self.data_send_sql = self.data_sql[self.stt1]
        x_goal, y_goal, z_goal, w_goal, idout = sql.doc_du_lieu_toado_robot(100)

        time.sleep(1)
        print('Gửi data: start')
        self.navigation_thread.clear_done_navigation_status() #reset bien done_navigation
        self.giaotoi = 0

        self.navigation_thread.id =  idout#300

        self.navigation_thread.quatrinh_move = True


        self.ros2_handle.goal_publisher.send_goal(x_goal, y_goal, z_goal, w_goal)

  def dinh_vi_fcn(self):

  
        self.navigation_thread.dang_dinhvi_status = True
        self.dang_dinhvi_cnt = 0
        Uti.RobotSpeakWithPath('voice_hmi_new/dinhvi.wav')
    
        
     
    
        #self.run_file_code_thread.duongdan = 2
        print('[dinh_vi_fcn]---------------------------BAT DAU QUA TRINH DINH VI------------------------')

        #B1: TIM HINH TRONG MAU VANG TREN CAM: NEU CO DUONG TRON THI TINH TAM VA CAP NHAT VI TRI
        counterMove = 0
        timeReadcame = 10
        cricle_yes = False
        radius_step = 0.5
        yaw_step = 45
        radius_current = 0.0
        radius_step = 0.5
        yaw_index = 0
        yaw_current = 0
       # yaw_list = [0, 45, 90, 135, 180, -135, -90, -45]
        yaw_list = [0, 45, 90, 135, 180, 225, 270, 315]
        result = False
        while (not result) and (counterMove < timeReadcame): # quay lai qua trinh doc came
            #if self.navigation_thread.quatrinh_move == False:
                            #time.sleep(2)
            rdeg, pdeg, yawdeg = self.quaternion_to_euler(self.ros2_handle.odom_listener.data_odom[0],self.ros2_handle.odom_listener.data_odom[1],
             0,self.ros2_handle.odom_listener.data_odom[3])
          
            print(f"[DEBUG] Góc yaw đầu vào: {yawdeg} độ")
        #self.sub_win1.uic.terminal_2.setPlainText(str(yawdeg))
       # print("GOC YAWdeg now: ", yawdeg)
            print(f"X:{self.ros2_handle.odom_listener.data_odom[0]}")
            print(f"Y:{self.ros2_handle.odom_listener.data_odom[1]}")
            print(f"Z:{self.ros2_handle.odom_listener.data_odom[2]}")
            print(f"W:{self.ros2_handle.odom_listener.data_odom[3]}")
            result  = self.setup_dv.test_image(200, yawdeg, self.capC, self.cap_qr)
            
            result_simlar = 0
            x_best, y_best, yaw_best = 0,0,0
            for idx, (coords, similarity) in enumerate(result[:1]):
                x_best, y_best, yaw_best, result_simlar = coords[0], coords[1], coords[2],similarity
                # result_text += f"({coords[0]}, {coords[1]}, {coords[2]}, {coords[3]}) "
                # result_text += f"{similarity:.6f} \n"
                

            # rdeg, pdeg, yawdeg = Uti.quaternion_to_euler(self.ros2_handle.odom_listener.data_odom[0],self.ros2_handle.odom_listener.data_odom[1], self.ros2_handle.odom_listener.data_odom[2], self.ros2_handle.odom_listener.data_odom[3])
            # yess, cx, cy, radius = Uti.dieuchinhvaTimduongtron(camID=headCamera, yaw_ros= yawdeg)
            if result_simlar > RobConf.DO_TUONG_DONG_CAM_DINHVIN:
                yess = True
            else:
                yess = False

            if yess:
                # Tinh duoc vi tri hinhtron                    
                cricle_yes = True
                # Chinh Pose he thong
                    # doc thong so ti le cam/m va vi tri home
                    # Đọc các giá trị từ tệp văn bản
                xx, yy = x_best,y_best
           
                print(f'[dinh_vi_fcn]Vi tri Diem hien tai cho x: {xx}, y = {yy}.')
                self.navigation_thread.user_set_PoseXY(xx, yy)
                result = True
                # print(f"x_dinhvi: {xx};  y_dinhvi: {yy}")
                # THONG BAO TAT CAC POPUP
                #self.run_file_code_thread.duongdan = 2
                #GHI VI TRI HIEN TAI VAO FILE
                
                print('[dinh_vi_fcn]------------------- ĐÃ LẤY XONG TỌA ĐỘ ---------------------')
                # time.sleep(3)
              #  Uti.writePose2File(filePath = file_path_pose_data,x=xx,y=yy,z=self.ros2_handle.odom_listener.data_odom[2],w=self.ros2_handle.odom_listener.data_odom[3])
                #with open('file_text_odom/odom_data.txt', 'w') as file:
                #    file.write(f'x: {xx}, y: {yy}, z: {self.ros2_handle.odom_listener.data_odom[2]}, w: {self.ros2_handle.odom_listener.data_odom[3]}\n')
                print(f'[dinh_vi_fcn] x: {xx}, y: {yy}, z: {self.ros2_handle.odom_listener.data_odom[2]}, w: {self.ros2_handle.odom_listener.data_odom[3]}','-pose cam')
                print("[dinh_vi_fcn]DINH VI XONG!!!!!")
                break
            elif (yess == False ) and (self.ht_chutrinh_dv == True) : #NEU KHONG CO DUOI N LAN THI QUA B2: GOI LENH DI CHUYEN ROBOT DEN VI TRI MONG MUON, CHO HE THONG DUNG, TIM HINH TRON MAU VANG
                # goi lenh di chuyen robot den vi tri moi
                # di chuyen den vi tri mong muon
                # tinh x, y mong muon   
                # cho he thong dung
                #kiem tra robot da toi diem mong muon chua:                    
              #  yaw_current = yaw_current +  yaw_step
                yaw_current = yaw_list[yaw_index]
               # if(yaw_current >= 360):
               #     radius_current = radius_current+radius_step
              #      yaw_current = (yaw_current + yaw_step) % 360
                   # yaw_current = 0
             
                
                x_desired = radius_current*np.cos(math.radians(yaw_current))
                y_desired = radius_current*np.sin(math.radians(yaw_current))
                z_desired = 0
              #  yaw_desired = math.radians(0)
                yaw_desired = math.radians(yaw_current)
                qx, qy, qz, qw = self.euler_to_quaternion(yaw_desired,0,0)
                w_desired = qw 
                print(f'[dinh_vi_fcn]-------Tim duong tron, Di chuyen den vi tri nay: goc = {yaw_current}, x = {x_desired}, y = {y_desired}')
                #self.desired_move(x_desired=x_desired, y_desired=y_desired, z_desired=qz, w_desired=qw,id_desired=200)      
                self.ros2_handle.goal_publisher.send_goal( x = x_desired, y = y_desired, z = z_desired, w = w_desired)
                print('[dinh_vi_fcn]CHO ROBOT TOI DIEM MOI XONG')
            #    time.sleep(5)
               # counterMove += 1
                if counterMove == 0:
                    print("[DEBUG] Bỏ qua ảnh đầu tiên do lệch góc")
                    time.sleep(0.5)
                    counterMove += 1
                    continue
                # Cập nhật vòng lặp
                yaw_index += 1
                if yaw_index >= len(yaw_list):
                    yaw_index = 0
                    radius_current += radius_step
        #NEU KHONG CO TREN N LAN THI THOAT RA VA THONG BAO BANG HINH ANH VA GIONG NOI
        self.navigation_thread.dang_dinhvi_status = False 

        if not cricle_yes:
            print("DINH VI THAT BAI")
          #  Uti.RobotSpeakWithPath('khongtimthayhinhtron.mp3')
            self.label_status.setText("Robot định vị thất bại")
            return False
        else:
            print("DINH VI XONG HOAN TOAN")
            self.label_status.setText("Robot định vị thành công")
            # Đóng cửa sổ thông báo sau khi hoàn thành
            #msg_box.close()
            return result 
    
      def dinh_vi_fcn(self):
    self.navigation_thread.dang_dinhvi_status = True
    self.dang_dinhvi_cnt = 0
    Uti.RobotSpeakWithPath('voice_hmi_new/dinhvi.wav')

    print('[dinh_vi_fcn]---------------------------BAT DAU QUA TRINH DINH VI------------------------')

    counterMove = 0
    timeReadcame = 10
    cricle_yes = False
    radius_step = 0.5
    yaw_step = 45
    radius_current = 0.0
    yaw_index = 0
    yaw_current = 0
    yaw_list = [0, 45, 90, 135, 180, 225, 270, 315]
    result = False

    while (not result) and (counterMove < timeReadcame):
        rdeg, pdeg, yawdeg = self.quaternion_to_euler(
            self.ros2_handle.odom_listener.data_odom[0],
            self.ros2_handle.odom_listener.data_odom[1],
            0,
            self.ros2_handle.odom_listener.data_odom[3]
        )

        print(f"[DEBUG] Góc yaw đầu vào: {yawdeg} độ")
        print(f"X:{self.ros2_handle.odom_listener.data_odom[0]}")
        print(f"Y:{self.ros2_handle.odom_listener.data_odom[1]}")
        print(f"Z:{self.ros2_handle.odom_listener.data_odom[2]}")
        print(f"W:{self.ros2_handle.odom_listener.data_odom[3]}")

        result_simlar = 0
        x_best, y_best, yaw_best = 0, 0, 0
        yess = False

        if yess:
            cricle_yes = True
            xx, yy = x_best, y_best

            print(f'[dinh_vi_fcn]Vi tri Diem hien tai cho x: {xx}, y = {yy}.')
            result = True

            print('[dinh_vi_fcn]------------------- ĐÃ LẤY XONG TỌA ĐỘ ---------------------')
            print(f'[dinh_vi_fcn] x: {xx}, y: {yy}, z: {self.ros2_handle.odom_listener.data_odom[2]}, w: {self.ros2_handle.odom_listener.data_odom[3]} -pose cam')
            print("[dinh_vi_fcn]DINH VI XONG!!!!!")
            break

        elif (yess == False) and (self.ht_chutrinh_dv == True):
            yaw_current = yaw_list[yaw_index]

            x_desired = radius_current * math.cos(math.radians(yaw_current))
            y_desired = radius_current * math.sin(math.radians(yaw_current))
            z_desired = 0
            yaw_desired = math.radians(yaw_current)
            qx, qy, qz, qw = self.euler_to_quaternion(yaw_desired, 0, 0)
            w_desired = qw

            print(f'[dinh_vi_fcn]-------Tim duong tron, Di chuyen den vi tri: goc = {yaw_current}, x = {x_desired:.2f}, y = {y_desired:.2f}')
            self.ros2_handle.goal_publisher.send_goal(x=x_desired, y=y_desired, z=z_desired, w=w_desired)

            # Chờ robot đến nơi + thêm thời gian để quét hình (2 giây)
            time.sleep(2)

            yaw_index += 1
            if yaw_index >= len(yaw_list):
                yaw_index = 0
                radius_current += radius_step
                print(f"[dinh_vi_fcn]--- Tăng bán kính lên: {radius_current} ---")

        counterMove += 1  # đảm bảo không lặp vô hạn nếu không tìm thấy

    print('[dinh_vi_fcn] KẾT THÚC QUÁ TRÌNH ĐỊNH VỊ')


def dinh_vi_fcn(self):
       
        self.dang_dinhvi_cnt = 0
        Uti.RobotSpeakWithPath('voice_hmi_new/dinhvi.wav')

        print('[dinh_vi_fcn]---------------------------BẮT ĐẦU QUÁ TRÌNH ĐỊNH VỊ------------------------')

        yaw_list_deg = [45,180,225]  # Danh sách góc yaw bạn muốn robot hướng tới (đơn vị độ)
        x_desired = 0.0
        y_desired = 0.0
        z_desired = 0.0
        self.current_yaw_index = 0  # Theo dõi góc yaw hiện tại trong danh sách
        print(f"Co :{len(yaw_list_deg)} goc")
        while self.current_yaw_index < len(yaw_list_deg):
         
            rdeg, pdeg, yawdeg = self.quaternion_to_euler(self.ros2_handle.odom_listener.data_odom[0],self.ros2_handle.odom_listener.data_odom[1], self.ros2_handle.odom_listener.data_odom[2],self.ros2_handle.odom_listener.data_odom[3] )

            print(f"Status done:{ self.ros2_handle.goal_publisher.dinhvi_vitri}")
            print(f"X:{self.ros2_handle.odom_listener.data_odom[0]}")
            print(f"Y:{self.ros2_handle.odom_listener.data_odom[1]}")
            print(f"Z:{self.ros2_handle.odom_listener.data_odom[2]}")
            print(f"W:{self.ros2_handle.odom_listener.data_odom[3]}")
            print(f"R:{rdeg}")
            print(f"P:{pdeg}")
            print(f"[DEBUG] Góc yaw hiện tại: {math.radians(yawdeg):.5f} rad ({yawdeg:.5f}°)")

     
            if  self.ros2_handle.goal_publisher.dinhvi_vitri == True:
                print(f'[dinh_vi_fcn] Robot đã đến góc {yaw_list_deg[self.current_yaw_index]}°.')
             #   self.di_dendiem_DV = False  # Reset để chuẩn bị cho góc tiếp theo
                self.current_yaw_index += 1 # Chuyển sang góc tiếp theo
            else :
                yaw_rad = math.radians(yaw_list_deg[self.current_yaw_index])
                qx_desired, qy_desired, qz_desired, qw_desired = self.euler_to_quaternion(yaw_rad, 0, 0)
            
                print(f'[dinh_vi_fcn]-------Di chuyển đến góc: {yaw_list_deg[self.current_yaw_index]}°, x = {x_desired:.2f}, y = {y_desired:.2f}, w = {qw_desired:.2f}')
                self.ros2_handle.goal_publisher.send_goal(x=x_desired, y=y_desired, z=z_desired, w=qw_desired)
        
                time.sleep(0.5) # Thêm một khoảng delay nhỏ để tránh gửi goal quá nhanh

                
              

        print('[dinh_vi_fcn] ================= KẾT THÚC QUÁ TRÌNH ĐỊNH VỊ =================')