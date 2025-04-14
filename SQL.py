import mysql.connector
from datetime import datetime

def doc_du_lieu_param_robot(input):
    """Đọc dữ liệu từ bảng parameter_robot và trả về giá trị hoặc danh sách các bản ghi."""
    
    # Thay thế phần này bằng mã kết nối cơ sở dữ liệu MySQL thực tế của bạn
    # Ví dụ sử dụng mysql.connector:
  

    try:
        mydb = mysql.connector.connect(
      
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()
    except mysql.connector.Error as err:
        print(f"Lỗi kết nối cơ sở dữ liệu: {err}")
        return None  # Trả về None nếu kết nối thất bại

    # Truy vấn để lấy dữ liệu
    select_query = "SELECT * FROM parameter_robot;"
    mycursor.execute(select_query)
    
    # Lấy tất cả các bản ghi
    results = mycursor.fetchall()

    # Mapping input keys to corresponding row indices and data types
    input_mapping = {
        'auto_vt_thang': (0, float),
        'auto_vt_xoay': (1, float),
        'manual_vt_thang': (2, float),
        'manual_vt_xoay': (3, float),
        'sai_so_kc': (4, float),
        'sai_so_goc': (5, float),
        'chieu_cao_led': (6, float),
        'username': (7, str),
        'password': (8, str),
        'volt_sac_auto': (9, str),
        'backC': (10, int),
        'headC': (11, int),
        'front_L': (12, int),
        'front_R': (13, int),
        'readall': (None, None)  # Special case for readall
    }

    for row in results:
        if input in input_mapping:
            index, data_type = input_mapping[input]
            if index is not None:
                return data_type(row[index])
            elif input == 'readall':
                return [
                    (float(row[0]), float(row[1]), float(row[2]),
                     float(row[3]), float(row[4]), float(row[5]),
                     float(row[6]), str(row[7]), str(row[8]), str(row[9]), int(row[10]),int(row[11]),int(row[12]),int(row[13]))
                ]

    return None  # Trả về None nếu input không hợp lệ hoặc không tìm thấy dữ liệu

def doc_du_lieu_robot_ros(input):
    """Đọc dữ liệu từ bảng parameter_robot và trả về giá trị hoặc danh sách các bản ghi."""
    
    # Thay thế phần này bằng mã kết nối cơ sở dữ liệu MySQL thực tế của bạn
    # Ví dụ sử dụng mysql.connector:
  

    try:
        mydb = mysql.connector.connect(
      
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()
    except mysql.connector.Error as err:
        print(f"Lỗi kết nối cơ sở dữ liệu: {err}")
        return None  # Trả về None nếu kết nối thất bại

    # Truy vấn để lấy dữ liệu
    select_query = "SELECT * FROM ros_config;"
    mycursor.execute(select_query)
    
    # Lấy tất cả các bản ghi
    results = mycursor.fetchall()

    # Mapping input keys to corresponding row indices and data types
    input_mapping = {
        'default_map_name': (0, str),
        'delay_time': (1, float),
    }

    for row in results:
        if input in input_mapping:
            index, data_type = input_mapping[input]
            if index is not None:
                return data_type(row[index])
            elif input == 'readall':
                return [
                    (float(row[0]), float(row[1]))
                ]

    return None  # Trả về None nếu input không hợp lệ hoặc không tìm thấy dữ liệu
def update_du_lieu_robot_ros(data):
    """Cập nhật dữ liệu vào bảng ros_config"""

    if not isinstance(data, dict):
        print("Dữ liệu đầu vào không hợp lệ.")
        return False

    try:
        mydb = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()

        update_query = "UPDATE ros_config SET "
        update_values = []
        for key, value in data.items():
            update_query += f"{key} = %s, "
            update_values.append(value)

        update_query = update_query[:-2] + ";"  # Remove trailing comma and space
        mycursor.execute(update_query, update_values)
        mydb.commit()

        if mycursor.rowcount > 0:
            print(f"Đã cập nhật {mycursor.rowcount} bản ghi.")
            return True
        else:
            print("Không có bản ghi nào được cập nhật.")
            return False

    except mysql.connector.Error as err:
        print(f"Lỗi cơ sở dữ liệu: {err}")
        if 'mydb' in locals() and mydb and mydb.is_connected():
            mydb.rollback()
        return False

    except Exception as e:
        print(f"Lỗi không xác định: {e}")
        if 'mydb' in locals() and mydb and mydb.is_connected():
            mydb.rollback()
        return False

    finally:
        if 'mycursor' in locals() and mycursor:
            mycursor.close()
        if 'mydb' in locals() and mydb and mydb.is_connected():
            mydb.close()
def doc_du_lieu_toado_robot(id=1):
    # Load dữ liệu từ MySQL dựa trên ID
    try:
        mydb = mysql.connector.connect(
      
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()
    except mysql.connector.Error as err:
        print(f"Lỗi kết nối cơ sở dữ liệu: {err}")
        return None  # Trả về None nếu kết nối thất bại

    # Truy vấn để lấy dữ liệu
    
    select_query = 'SELECT * FROM toado WHERE `IDban` = %s LIMIT 0, 1'  # Lấy chỉ một dòng dữ liệu
    mycursor.execute(select_query, (id,))
    result = mycursor.fetchone()  # Sử dụng fetchone() để lấy chỉ một dòng dữ liệu
    
    if result:
        x,y,z,w = result[0:4]
        idout = id
        return  (x,y,z,w, idout)
        print(f"Dữ liệu tọa độ robot (ID={idout}):")
        print(f"X: {x}, Y: {y}, Z: {z}, W: {w}")
    else:
        return None 

def kiem_tra_id_ton_tai(id_kiem_tra):
    try:
        mydb = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()

        select_query = "SELECT * FROM toado WHERE IDban = %s LIMIT 1"
        mycursor.execute(select_query, (id_kiem_tra,))
        result = mycursor.fetchone()

        return result is not None  # Trả về True nếu result không phải None, False nếu ngược lại
        
    except mysql.connector.Error as err:
        print(f"Lỗi kết nối cơ sở dữ liệu: {err}")
        return False  # Giả định ID không tồn tại nếu có lỗi

    finally:
        if 'mydb' in locals() and mydb.is_connected():
            mycursor.close()
            mydb.close()

def setupRobot_sql_delete():
    """Xóa tất cả dữ liệu trong bảng parameter_robot."""
    try:
        mydb = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()
        delete_query = "DELETE FROM parameter_robot;"
        mycursor.execute(delete_query)
        mydb.commit()  # Lưu thay đổi vào cơ sở dữ liệu
        print("Đã xóa tất cả dữ liệu trong bảng parameter_robot.")

    except mysql.connector.Error as err:
        print(f"Lỗi cơ sở dữ liệu: {err}")
        if 'mydb' in locals() and mydb and mydb.is_connected():
            mydb.rollback() #rollback in case of error.
        return None  # Trả về None nếu có lỗi

    finally:
        if 'mycursor' in locals() and mycursor:
            mycursor.close()
        if 'mydb' in locals() and mydb and mydb.is_connected():
            mydb.close() # Đóng kết nối cơ sở dữ liệu

def setupRobot_sql_save( aVx=0, aVw=0, mVx=0, mVw=0, ss_kc=0, ss_gx=0, led=0, user=0, pw=0, volt = 0, hC = 0, bC = 2, FL = 4, FR = 6):
    setupRobot_sql_delete()
    """Chèn tọa độ mới vào cơ sở dữ liệu mà không cần kiểm tra ID trùng lặp."""
    try:
        mydb = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()
      # Chuẩn bị dữ liệu để chèn
        data_to_insert = (aVx, aVw, mVx, mVw, ss_kc, ss_gx, led, user, pw, volt, hC, bC, FL, FR)  # Thay 'username' và 'password' theo yêu cầu
        insert_query = """
        INSERT INTO parameter_robot 
        (`auto_vt_thang`, `auto_vt_xoay`, `manual_vt_thang`, `manual_vt_xoay`, 
        `sai_so_kc`, `sai_so_goc`, `chieu_cao_led`, `username`, `password`, `volt_sac_auto`, `backC`, `headC`, `front_L`, `front_R`) 
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s);
        """
    
        mycursor.execute(insert_query, data_to_insert)
        mydb.commit()  # Lưu thay đổi vào cơ sở dữ liệu
        print("Đã chèn tọa độ mới vào cơ sở dữ liệu.")

    except mysql.connector.Error as err:
        print(f"Lỗi cơ sở dữ liệu: {err}")
        if 'mydb' in locals() and mydb and mydb.is_connected():
            mydb.rollback() #rollback in case of error.
        return None  # Trả về None nếu có lỗi

    finally:
        if 'mycursor' in locals() and mycursor:
            mycursor.close()
        if 'mydb' in locals() and mydb and mydb.is_connected():
            mydb.close() # Đóng kết nối cơ sở dữ liệu
  
    return True


   
    
def doc_du_lieu_list(input):
    """Đọc dữ liệu từ bảng LIST và trả về giá trị hoặc danh sách các bản ghi."""

    try:
        mydb = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()
    except mysql.connector.Error as err:
        print(f"Lỗi kết nối cơ sở dữ liệu: {err}")
        return None

    # Truy vấn để lấy dữ liệu từ bảng LIST
    select_query = "SELECT * FROM LIST;"
    mycursor.execute(select_query)

    # Lấy tất cả các bản ghi
    results = mycursor.fetchall()

    # Mapping input keys to corresponding row indices and data types for LIST table
    input_mapping = {
        'ID': (0, int),
        'Name': (1, str),
        'Room': (2, str),
        'DateTime_Checkin': (3, datetime),
        'DateTime_CheckOut': (4, datetime),
        'Image_Checkin': (5, str),
        'Image_Checkout': (6, str),
        'readall': (None, None)  # Special case for readall
    }

    for row in results:
        if input in input_mapping:
            index, data_type = input_mapping[input]
            if index is not None:
                return data_type(row[index])
            elif input == 'readall':
                return [
                    (int(row[0]), str(row[1]), str(row[2]),
                     row[3], row[4], str(row[5]), str(row[6]))
                    for row in results
                ]

    return None  # Trả về None nếu input không hợp lệ hoặc không tìm thấy dữ liệu

def delete_all_data_list():
    """Xóa tất cả dữ liệu từ bảng LIST."""
    try:
        mydb = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()
    except mysql.connector.Error as err:
        print(f"Lỗi kết nối cơ sở dữ liệu: {err}")
        return False

    try:
        # Câu lệnh SQL để xóa tất cả dữ liệu từ bảng LIST
        delete_query = "DELETE FROM LIST;"
        mycursor.execute(delete_query)
        mydb.commit()
        print(f"Đã xóa thành công {mycursor.rowcount} bản ghi từ bảng LIST.")
        return True
    except mysql.connector.Error as err:
        print(f"Lỗi khi xóa dữ liệu từ bảng LIST: {err}")
        mydb.rollback()
        return False
    finally:
        if mydb.is_connected():
            mycursor.close()
            mydb.close()