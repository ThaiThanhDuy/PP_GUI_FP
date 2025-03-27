# import cv2
# import os
# import sqlite3
# import time

# headCame = 2
# ChargCame = 0
# faceCameT = 4
# faceCameN = 6

# class ImageDatabase:
#     def __init__(self, db_path, image_dir):
#         self.db_path = db_path
#         self.image_dir = image_dir
#         self.conn = sqlite3.connect(self.db_path)
#         self.create_table()

#     def create_table(self):
#         # Xóa bảng nếu đã tồn tại để đảm bảo cấu trúc đúng
#         self.conn.execute("DROP TABLE IF EXISTS images")

#         # Tạo bảng với cột x, y, z
#         query = """
#         CREATE TABLE IF NOT EXISTS images (
#             id INTEGER PRIMARY KEY AUTOINCREMENT,
#             file_name TEXT,
#             x REAL,
#             y REAL,
#             z REAL
#         )
#         """
#         self.conn.execute(query)
#         self.conn.commit()

#     def add_image(self, file_name, x, y, z, id):
#         query = "INSERT INTO images (file_name, x, y, z) VALUES (?, ?, ?, ?)"
#         self.conn.execute(query, (file_name, x, y, z, id))
#         self.conn.commit()

#     def load_images(self):
#         query = "SELECT file_name, x, y, z FROM images"
#         cursor = self.conn.execute(query)
#         data = cursor.fetchall()
#         return data
    
#     def delete_image(self, image_id):
#         query = "DELETE FROM images WHERE id = ?"
#         self.conn.execute(query, (image_id,))
#         self.conn.commit()
    
#     def update_data(self, file_name, x, y, z):
#         # Kiểm tra xem bản ghi đã tồn tại chưa
#         query_check = "SELECT COUNT(*) FROM images WHERE file_name = ?"
#         cursor = self.conn.execute(query_check, (file_name,))
#         count = cursor.fetchone()[0]
        
#         if count > 0:
#             # Cập nhật dữ liệu nếu bản ghi đã tồn tại
#             query_update = "UPDATE images SET x = ?, y = ?, z = ? WHERE file_name = ?"
#             self.conn.execute(query_update, (x, y, z, file_name))
#         else:
#             # Thêm dữ liệu nếu bản ghi chưa tồn tại
#             query_insert = "INSERT INTO images (file_name, x, y, z) VALUES (?, ?, ?, ?)"
#             self.conn.execute(query_insert, (file_name, x, y, z))
        
#         self.conn.commit()

#     def close(self):
#         self.conn.close()

# def capture_image(ip_camera_url, save_path, image_name):
#     cap = cv2.VideoCapture(ip_camera_url)
#     ret, frame = cap.read()
#     if ret:
#         image_path = os.path.join(save_path, image_name)
#         cv2.imwrite(image_path, frame)
#     cap.release()
#     return ret

# def main():
#     # Set up camera X Y
#     db_path = './dvC_image_database_XY.db'
#     image_dir = './dvC_images_XY'

#     # Setup camera Z
#     # db_path = './dvC_image_database_Z.db'
#     # image_dir = './dvC_images_Z'

#     ip_camera_url = headCame
    
#     if not os.path.exists(image_dir):
#         os.makedirs(image_dir)

#     # Xóa cơ sở dữ liệu cũ nếu tồn tại
#     # if os.path.exists(db_path):
#     #     os.remove(db_path)

#     image_db = ImageDatabase(db_path, image_dir)

#     while True:
#         user_input = input("Type 'OK' to capture an image, 'Update' to update data, or 'Done' to finish: ").strip().lower()
#         if user_input == 'done':
#             break

#         if user_input == 'ok':
#             image_name = '{}.jpg'.format(int(time.time()))
#             if capture_image(ip_camera_url, image_dir, image_name):
#                 print(f"Captured and saved {image_name}")

#                 x = float(input("Enter x coordinate: "))
#                 y = float(input("Enter y coordinate: "))
#                 z = float(input("Enter z coordinate: "))
#                 id = float(input("Enter id coordinate: "))

#                 # Thêm dữ liệu mới vào cơ sở dữ liệu
#                 image_db.add_image(image_name, x, y, z, id)
#                 print(f"Stored {image_name} with coordinates ({x}, {y}, {z}, {id})")
#             else:
#                 print("Failed to capture image.")
#         elif user_input == 'update':
#             image_name = input("Enter the image file name to update: ")
#             x = float(input("Enter new x coordinate: "))
#             y = float(input("Enter new y coordinate: "))
#             z = float(input("Enter new z coordinate: "))

#             # Cập nhật dữ liệu nếu ảnh đã tồn tại
#             image_db.update_data(image_name, x, y, z)
#             print(f"Updated {image_name} with coordinates ({x}, {y}, {z})")
#         else:
#             print("Invalid input. Please type 'OK', 'Update', or 'Done'.")

#     image_db.close()


# if __name__ == "__main__":
#     main()


import cv2
import os
import sqlite3
import time

headCame = 2
ChargCame = 0
faceCameT = 4
faceCameN = 6

def create_table(conn):
    # Xóa bảng nếu đã tồn tại để đảm bảo cấu trúc đúng
    conn.execute("DROP TABLE IF EXISTS images")

    # Tạo bảng với cột x, y, z, id
    query = """
    CREATE TABLE IF NOT EXISTS images (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        file_name TEXT,
        x REAL,
        y REAL,
        z REAL,
        image_id REAL
    )
    """
    conn.execute(query)
    conn.commit()

def add_image(conn, file_name, x, y, z, image_id):
    query = "INSERT INTO images (file_name, x, y, z, image_id) VALUES (?, ?, ?, ?, ?)"
    conn.execute(query, (file_name, x, y, z, image_id))
    conn.commit()

def load_images(conn):
    query = "SELECT file_name, x, y, z, image_id FROM images"
    cursor = conn.execute(query)
    data = cursor.fetchall()
    return data

def delete_image(conn, image_id):
    query = "DELETE FROM images WHERE id = ?"
    conn.execute(query, (image_id,))
    conn.commit()

def update_data(conn, file_name, x, y, z):
    # Kiểm tra xem bản ghi đã tồn tại chưa
    query_check = "SELECT COUNT(*) FROM images WHERE file_name = ?"
    cursor = conn.execute(query_check, (file_name,))
    count = cursor.fetchone()[0]
    
    if count > 0:
        # Cập nhật dữ liệu nếu bản ghi đã tồn tại
        query_update = "UPDATE images SET x = ?, y = ?, z = ? WHERE file_name = ?"
        conn.execute(query_update, (x, y, z, file_name))
    else:
        # Thêm dữ liệu nếu bản ghi chưa tồn tại
        query_insert = "INSERT INTO images (file_name, x, y, z) VALUES (?, ?, ?, ?)"
        conn.execute(query_insert, (file_name, x, y, z))
    
    conn.commit()

def capture_image(ip_camera_url, save_path, image_name):
    cap = cv2.VideoCapture(ip_camera_url)
    ret, frame = cap.read()
    if ret:
        image_path = os.path.join(save_path, image_name)
        cv2.imwrite(image_path, frame)
    cap.release()
    return ret

def main():
    # Set up camera X Y
    db_path = './dvC_image_database_XY.db'
    image_dir = './dvC_images_XY'

    ip_camera_url = headCame
    
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)

    conn = sqlite3.connect(db_path)
    create_table(conn)

    while True:
        user_input = input("Type 'OK' to capture an image, 'Update' to update data, or 'Done' to finish: ").strip().lower()
        if user_input == 'done':
            break

        if user_input == 'ok':
            image_name = '{}.jpg'.format(int(time.time()))
            if capture_image(ip_camera_url, image_dir, image_name):
                print(f"Captured and saved {image_name}")

                x = float(input("Enter x coordinate: "))
                y = float(input("Enter y coordinate: "))
                z = float(input("Enter z coordinate: "))
                image_id = float(input("Enter id coordinate: "))

                # Thêm dữ liệu mới vào cơ sở dữ liệu
                add_image(conn, image_name, x, y, z, image_id)
                print(f"Stored {image_name} with coordinates ({x}, {y}, {z}, {image_id})")
            else:
                print("Failed to capture image.")
        elif user_input == 'update':
            image_name = input("Enter the image file name to update: ")
            x = float(input("Enter new x coordinate: "))
            y = float(input("Enter new y coordinate: "))
            z = float(input("Enter new z coordinate: "))

            # Cập nhật dữ liệu nếu ảnh đã tồn tại
            update_data(conn, image_name, x, y, z)
            print(f"Updated {image_name} with coordinates ({x}, {y}, {z})")
        elif user_input == "load":
            data = load_images(conn)
            print(data)
        else:
            print("Invalid input. Please type 'OK', 'Update', or 'Done'.")

    conn.close()

if __name__ == "__main__":
    main()
