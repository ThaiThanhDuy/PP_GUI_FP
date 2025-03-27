import cv2
import numpy as np

# Hàm tính toán khoảng cách và góc lệch
def calculate_shift(kp1, kp2, matches):
    if len(matches) == 0:
        return None, None
    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
    dx = np.mean(pts2[:, 0] - pts1[:, 0])
    dy = np.mean(pts2[:, 1] - pts1[:, 1])
    return dx, dy

# Đọc hình ảnh tham chiếu và hình ảnh hiện tại
img_ref = cv2.imread('images/image0.jpg', cv2.IMREAD_GRAYSCALE)  # Hình ảnh tham chiếu tại điểm A
img_cur = cv2.imread('images/image2.jpg', cv2.IMREAD_GRAYSCALE)    # Hình ảnh tại vị trí hiện tại

# Trích xuất đặc trưng bằng SIFT
sift = cv2.SIFT_create()
kp_ref, des_ref = sift.detectAndCompute(img_ref, None)
kp_cur, des_cur = sift.detectAndCompute(img_cur, None)

# So khớp các đặc trưng bằng BFMatcher
bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
matches = bf.match(des_ref, des_cur)
matches = sorted(matches, key=lambda x: x.distance)

# Tính toán khoảng cách và góc lệch
dx, dy = calculate_shift(kp_ref, kp_cur, matches)

if dx is not None and dy is not None:
    print(f"Khoảng cách lệch: dx = {dx}, dy = {dy}")
else:
    print("Không tìm thấy đặc trưng khớp")

# Vẽ các đặc trưng khớp
img_matches = cv2.drawMatches(img_ref, kp_ref, img_cur, kp_cur, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
cv2.imshow('Matches', img_matches)
cv2.waitKey(0)
cv2.destroyAllWindows()
