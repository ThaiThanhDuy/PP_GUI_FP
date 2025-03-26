


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