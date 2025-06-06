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
def chen_du_lieu_list(data):
    """Chèn một bản ghi mới vào bảng LIST, bao gồm cả ID."""
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

    # Kiểm tra xem dữ liệu đầu vào có đúng định dạng không
    if not isinstance(data, dict):
        print("Dữ liệu đầu vào phải là một dictionary.")
        return False

    required_keys = ['ID', 'Name', 'Room', 'DateTime_Checkin', 'DateTime_CheckOut']
    if not all(key in data for key in required_keys):
        print(f"Vui lòng cung cấp đầy đủ các khóa sau: {required_keys}")
        return False

    # Lấy dữ liệu từ dictionary
    record_id = data.get('ID')
    name = data.get('Name')
    room = data.get('Room')
    datetime_checkin = data.get('DateTime_Checkin')
    datetime_checkout = data.get('DateTime_CheckOut')
    image_checkin = data.get('Image_Checkin')
    image_checkout = data.get('Image_Checkout')

    # Tạo câu lệnh SQL INSERT
    insert_query = "INSERT INTO LIST (ID, Name, Room, DateTime_Checkin, DateTime_CheckOut, Image_Checkin, Image_Checkout) VALUES (%s, %s, %s, %s, %s, %s, %s)"
    values = (record_id, name, room, datetime_checkin, datetime_checkout, image_checkin, image_checkout)

    try:
        mycursor.execute(insert_query, values)
        mydb.commit()
        print(f"Đã chèn thành công {mycursor.rowcount} bản ghi với ID = {record_id}.")
        return True
    except mysql.connector.Error as err:
        print(f"Lỗi khi chèn dữ liệu: {err}")
        mydb.rollback()
        return False
    finally:
        if mydb.is_connected():
            mycursor.close()
            mydb.close()
def doc_du_lieu_toado(input_key):
    """Đọc dữ liệu từ bảng toado và trả về giá trị hoặc danh sách các bản ghi."""

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

    # Truy vấn để lấy dữ liệu từ bảng toado
    select_query = "SELECT toadoX, toadoY, toadoZ, toadoW, IDban FROM toado;"
    mycursor.execute(select_query)

    # Lấy tất cả các bản ghi
    results = mycursor.fetchall()

    # Mapping input keys to corresponding row indices and data types for toado table
    input_mapping = {
        'toadoX': (0, float),
        'toadoY': (1, float),
        'toadoZ': (2, float),
        'toadoW': (3, float),
        'IDban': (4, int),
        'readall': (None, None)  # Special case for readall
    }

    if input_key in input_mapping:
        index, data_type = input_mapping[input_key]
        if index is not None:
            # Duyệt qua các hàng và trả về giá trị đầu tiên tìm thấy
            for row in results:
                return data_type(row[index])
        elif input_key == 'readall':
            return [
                (float(row[0]), float(row[1]), float(row[2]),
                 float(row[3]), int(row[4]))
                for row in results
            ]

    return None  # Trả về None nếu input không hợp lệ hoặc không tìm thấy dữ liệu

def doc_name_theo_id(id: int):
    """
    Đọc dữ liệu 'Name' từ bảng 'nameLocation' dựa trên 'ID'.

    Args:
        id (int): ID của bản ghi cần đọc.

    Returns:
        tuple: (Name, ID) nếu tìm thấy, ngược lại là None.
    """
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

    select_query = 'SELECT Name, ID FROM nameLocation WHERE ID = %s LIMIT 1'
    mycursor.execute(select_query, (id,))
    result = mycursor.fetchone()

    mycursor.close()
    mydb.close()

    if result:
        name_out = result[0]
        id_out = result[1]
        # Bỏ dòng print này nếu bạn chỉ muốn hàm trả về giá trị
        # print(f"Dữ liệu từ nameLocation (ID={id_out}):")
        # print(f"Name: {name_out}")
        return (name_out, id_out)
    else:
        # Bỏ dòng print này nếu bạn chỉ muốn hàm trả về None
        # print(f"Không tìm thấy dữ liệu cho ID = {id}")
        return None

def get_location_name_from_db(id: int):
    """
    Đọc dữ liệu 'Name' từ bảng 'nameLocation' dựa trên 'ID'.
    Trả về Name nếu tìm thấy, ngược lại là None.
    """
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

    select_query = 'SELECT Name FROM nameLocation WHERE ID = %s LIMIT 1'
    mycursor.execute(select_query, (id,))
    result = mycursor.fetchone()

    mycursor.close()
    mydb.close()

    if result:
        return result[0]  # Chỉ trả về Name
    else:
        return None


def update_or_insert_location_name(id: int, name: str):
    """
    Cập nhật dữ liệu 'Name' cho một 'ID' cụ thể trong bảng 'nameLocation'.
    Nếu 'ID' đã tồn tại, nó sẽ cập nhật 'Name'.
    Nếu 'ID' chưa tồn tại, nó sẽ chèn một hàng mới.
    **Lưu ý: Cách này kém hiệu quả hơn và có thể gặp lỗi trong môi trường đa luồng
    nếu cột ID không phải là PRIMARY KEY/UNIQUE KEY.**
    """
    mydb = None
    mycursor = None
    try:
        mydb = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()

        # Bước 1: Kiểm tra xem ID đã tồn tại chưa
        # Lưu ý: Chúng ta không thể dùng get_location_name_from_db ở đây vì nó sẽ đóng kết nối.
        # Chúng ta cần giữ kết nối mở cho thao tác tiếp theo.
        check_query = 'SELECT COUNT(*) FROM nameLocation WHERE ID = %s'
        mycursor.execute(check_query, (id,))
        exists = mycursor.fetchone()[0] > 0

        if exists:
            # Nếu ID tồn tại, thực hiện UPDATE
            sql_query = 'UPDATE nameLocation SET Name = %s WHERE ID = %s'
            data = (name, id)
            action = "cập nhật"
        else:
            # Nếu ID không tồn tại, thực hiện INSERT
            sql_query = 'INSERT INTO nameLocation (ID, Name) VALUES (%s, %s)'
            data = (id, name)
            action = "thêm mới"

        mycursor.execute(sql_query, data)
        mydb.commit() # Xác nhận các thay đổi vào cơ sở dữ liệu

        if mycursor.rowcount > 0:
            print(f"ID {id}: Dữ liệu đã được {action} thành công. Số hàng ảnh hưởng: {mycursor.rowcount}")
        else:
            print(f"ID {id}: Không có sự thay đổi nào được thực hiện (có thể Name đã giống hoặc không tìm thấy ID để cập nhật).")

    except mysql.connector.Error as err:
        print(f"Lỗi khi {action} dữ liệu vào cơ sở dữ liệu: {err}")
        return False
    finally:
        if mycursor:
            mycursor.close()
        if mydb:
            mydb.close()
    return True
def clear_name_location_table():
    """
    Xóa toàn bộ dữ liệu (các hàng) khỏi bảng 'nameLocation' mà không xóa bảng đó.
    Sử dụng TRUNCATE TABLE để xóa nhanh và hiệu quả.
    """
    mydb = None
    mycursor = None
    try:
        mydb = mysql.connector.connect(
            user="robot",
            password="12345678",
            host="127.0.0.1",
            database="sql_rsr"
        )
        mycursor = mydb.cursor()

        # Câu lệnh TRUNCATE TABLE sẽ xóa tất cả các hàng nhưng giữ lại cấu trúc bảng.
        # Nó cũng sẽ reset các AUTO_INCREMENT counter về 0.
        sql_query = "TRUNCATE TABLE nameLocation"
        mycursor.execute(sql_query)
        mydb.commit() # Xác nhận thay đổi vào database

        print("Đã xóa toàn bộ dữ liệu (các hàng) khỏi bảng 'nameLocation' thành công. Bảng vẫn còn tồn tại.")
 
        return True

    except mysql.connector.Error as err:
        error_message = f"Lỗi khi xóa dữ liệu bảng 'nameLocation': {err}"
        print(error_message)

        return False
    finally:
        if mycursor:
            mycursor.close()
        if mydb:
            mydb.close()
def doc_du_lieu_ID(input_key):
    """Đọc dữ liệu từ bảng toado và trả về giá trị hoặc danh sách các bản ghi."""

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

    # Truy vấn để lấy dữ liệu từ bảng toado
    select_query = "SELECT ID, Name FROM nameLocation;"
    mycursor.execute(select_query)

    # Lấy tất cả các bản ghi
    results = mycursor.fetchall()

    # Mapping input keys to corresponding row indices and data types for toado table
    input_mapping = {
        'ID': (0, int),
        'Name': (1, str),
        'readall': (None, None)  # Special case for readall
    }

    if input_key in input_mapping:
        index, data_type = input_mapping[input_key]
        if index is not None:
            # Duyệt qua các hàng và trả về giá trị đầu tiên tìm thấy
            for row in results:
                return data_type(row[index])
        elif input_key == 'readall':
            return [
                (int(row[0]), str(row[1]))
                for row in results
            ]

    return None  # Trả về None nếu input không hợp lệ hoặc không tìm thấy dữ liệu