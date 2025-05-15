import cv2
import numpy as np
from othello import BLACK, WHITE, EMPTY, HW, HW2

BOARD_IMAGE_SIZE = 256
CELL_SIZE_MM = 29.0
DISC_HEIGHT_SHIFT_TOP = 5 #4.42 / (CELL_SIZE_MM * HW) * BOARD_IMAGE_SIZE
DISC_HEIGHT_SHIFT_BOTTOM = 13 #5.53 / (CELL_SIZE_MM * HW) * BOARD_IMAGE_SIZE

DEBUG_IMSHOW = True

# グローバル変数でクリックした座標を保存
default_points = [(185, 237), (78, 387), (510, 389), (410, 240)]

'''
def mouse_callback(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:  # 左クリック時
        points.append((x, y))
        print(f"Point {len(points)}: {x}, {y}")
        if len(points) == 4:
            print("4つの点を取得しました:", points)
'''

def recognize_disc_place(transformed):
    # 画像をHSVに変換
    hsv = cv2.cvtColor(transformed, cv2.COLOR_BGR2HSV)

    # 緑を抽出 参考: https://qiita.com/tanaka-a/items/fe6b95ae922b684021cc
    hsv = cv2.cvtColor(transformed, cv2.COLOR_BGR2HSV)
    lower = np.array([45,50,30])
    upper = np.array([90,255,255])
    green1 = cv2.inRange(hsv,lower,upper)
    lower = np.array([45,64,89])
    upper = np.array([90,255,255])
    green2 = cv2.inRange(hsv,lower,upper)
    color_mask = cv2.bitwise_or(green1,green2)
    # マスクを反転
    disc_mask = cv2.bitwise_not(color_mask)

    # デバッグ用にマスクと結果を表示
    if DEBUG_IMSHOW:
        cv2.imshow('disc_mask', disc_mask)

    # 上下左右のうち、どちらかの隣がオンならそのピクセルもオンにする
    kernel = np.array([[0, 1, 0],
                       [1, 1, 1],
                       [0, 1, 0]], dtype=np.uint8)
    disc_mask = cv2.dilate(disc_mask, kernel)

    # 周囲5ピクセル以内にしかオンのピクセルを持たない塊を削除
    contours, _ = cv2.findContours(disc_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 5 or y <= 5 or x + w >= disc_mask.shape[1] - 5 or y + h >= disc_mask.shape[0] - 5:
            cv2.drawContours(disc_mask, [contour], -1, 0, -1)

    '''
    # 縦方向に細長い要素と横方向に細長い要素を削除
    contours, _ = cv2.findContours(disc_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else float('inf')
        if x <= 5 or x + w >= disc_mask.shape[1] - 5:
            if h > w and aspect_ratio > 3:  # 縦長の細長い部分のみを消す
                cv2.drawContours(disc_mask, [contour], -1, 0, -1)
        elif y <= 5 or y + h >= disc_mask.shape[0] - 5:
            if w > h and aspect_ratio > 3:  # 横長の細長い部分のみを消す
                cv2.drawContours(disc_mask, [contour], -1, 0, -1)
    '''
                
    # L字のような形がdisc_mask内に存在すれば削除
    height, width = disc_mask.shape
    l_shapes = [
        np.array([[0, 0], [0, 50], [10, 50], [10, 10], [50, 10], [50, 0]], dtype=np.int32),  # 左上
        np.array([[width, 0], [width - 50, 0], [width - 50, 10], [width - 10, 10], [width - 10, 50], [width, 50]], dtype=np.int32),  # 右上
        np.array([[0, height], [0, height - 50], [10, height - 50], [10, height - 10], [50, height - 10], [50, height]], dtype=np.int32),  # 左下
        np.array([[width, height], [width - 50, height], [width - 50, height - 10], [width - 10, height - 10], [width - 10, height - 50], [width, height - 50]], dtype=np.int32)  # 右下
    ]
    for l_shape in l_shapes:
        cv2.fillPoly(disc_mask, [l_shape], 0)
    
    # マスクを適用して抽出
    #discs = cv2.bitwise_and(transformed, transformed, mask=disc_mask)

    # デバッグ用にマスクと結果を表示
    if DEBUG_IMSHOW:
        cv2.imshow('disc_mask_updated', disc_mask)

    # transformedの明るさが200以上のピクセルを抽出
    brightness_mask = cv2.inRange(cv2.cvtColor(transformed, cv2.COLOR_BGR2GRAY), 200, 255)
    white_mask = cv2.bitwise_and(disc_mask, brightness_mask)
    kernel = np.ones((11, 11), np.uint8)  # 5ピクセル拡大のためのカーネルサイズは (2*5+1, 2*5+1)
    expanded_brightness_mask = cv2.dilate(brightness_mask, kernel)
    black_mask = cv2.bitwise_and(disc_mask, cv2.bitwise_not(expanded_brightness_mask))
    
    if DEBUG_IMSHOW:
        cv2.imshow('Black Mask', black_mask)
        cv2.imshow('White Mask', white_mask)
    
    disc_masks = [black_mask, white_mask]
    disc_centers = []

    for i in range(2):
        mask = disc_masks[i]
        # 下辺以外の画像の端1ピクセルをマスクしないようにする
        mask[0, :] = 0
        #mask[-1, :] = 0
        mask[:, 0] = 0
        mask[:, -1] = 0
        # 距離変換を適用して、mask外からの距離を計算
        distance_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 5)

        # 距離を正規化して0-255の範囲にスケール
        normalized_distance = cv2.normalize(distance_transform, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # 距離画像を表示
        if DEBUG_IMSHOW:
            cv2.imshow('Distance ' + ('Black' if i == BLACK else 'White'), normalized_distance)

        # 局所的な最大値を持つピクセルを抽出
        kernel = np.ones((1 + 5 * 2, 1 + 5 * 2), np.uint8)  # 周囲5ピクセルの範囲で最大値を持つピクセルを抽出
        local_maxima = cv2.dilate(normalized_distance, kernel) == normalized_distance
        #local_maxima = cv2.dilate(normalized_distance, None) == normalized_distance
        local_maxima = np.uint8(local_maxima) * 255
        local_maxima = cv2.bitwise_and(local_maxima, disc_mask)
        # normalized_distanceの要素が70以上のマスクを作成
        threshold_mask = cv2.inRange(normalized_distance, 70, 255)
        local_maxima = cv2.bitwise_and(local_maxima, threshold_mask)
        if DEBUG_IMSHOW:
            cv2.imshow('Local Maxima ' + ('Black' if i == BLACK else 'White'), local_maxima)
        

        old_local_maxima = local_maxima

        for _ in range(3):
            disc_center = np.zeros_like(old_local_maxima, dtype=np.uint8)

            # local_maximaのすべてのオンのピクセルについて処理を行う
            for y, x in zip(*np.where(old_local_maxima > 0)):
                # そのピクセルから10ピクセル以内のオンのピクセルを取得
                mask = np.zeros_like(old_local_maxima, dtype=np.uint8)
                cv2.circle(mask, (x, y), 10, 255, -1)
                nearby_pixels = cv2.bitwise_and(old_local_maxima, mask)

                # nearby_pixelsのオンのピクセルの重心を計算
                coords = np.column_stack(np.where(nearby_pixels > 0))
                if len(coords) > 0:
                    centroid = np.mean(coords, axis=0).astype(int)
                    # 重心ピクセルをdisc_centerでオンにする
                    disc_center[centroid[0], centroid[1]] = 255
            old_local_maxima = disc_center
        
        disc_centers.append(disc_center)

        if DEBUG_IMSHOW:
            cv2.imshow('Disc Center ' + ('Black' if i == BLACK else 'White'), disc_center)


    disc_places = [[], []]
    for y in range(HW):
        for x in range(HW):
            leftupper_x = BOARD_IMAGE_SIZE / HW * x
            leftupper_y = round(BOARD_IMAGE_SIZE / HW * y + DISC_HEIGHT_SHIFT_TOP * (HW - 1 - y) / (HW - 1) + DISC_HEIGHT_SHIFT_BOTTOM * y / (HW - 1))
            rightbottom_x = leftupper_x + BOARD_IMAGE_SIZE / HW
            rightbottom_y = min(BOARD_IMAGE_SIZE, round(BOARD_IMAGE_SIZE / HW * (y + 1) + DISC_HEIGHT_SHIFT_TOP * (HW - 1 - (y + 1)) / (HW - 1) + DISC_HEIGHT_SHIFT_BOTTOM * (y + 1) / (HW - 1)))

            # 四角形の範囲を整数に変換
            leftupper_x = int(leftupper_x)
            leftupper_y = int(leftupper_y)
            rightbottom_x = int(rightbottom_x)
            rightbottom_y = int(rightbottom_y)

            for i in [WHITE, BLACK]:
                disc_center = disc_centers[i]

                # 四角形の中のdisc_centerのオンとなっているピクセルをリストで取得
                on_pixels = np.column_stack(np.where(disc_center[leftupper_y:rightbottom_y, leftupper_x:rightbottom_x] > 0))
                if len(on_pixels):
                    cy, cx = on_pixels[0]
                    cy += leftupper_y
                    cx += leftupper_x
                    disc_places[i].append([cx, cy])
                    break
    return disc_places

def get_points_single():
    ret, frame = cap.read()
    if not ret:
        print("Cannot receive a frame")
        return default_points

    # default_pointsの外側10ピクセルの四角形を計算
    points = np.array(default_points, dtype=np.float32)
    expanded_points = points.copy()
    for i, point in enumerate(points):
        prev_point = points[i - 1]
        next_point = points[(i + 1) % len(points)]
        direction = np.mean([point - prev_point, point - next_point], axis=0)
        direction = direction / np.linalg.norm(direction) * (5 if i == 0 or i == 3 else 10)  # iが0か3のときは5ピクセル拡張
        expanded_points[i] += direction

    # 射影変換行列を計算
    dst_points = np.array([
        [0, 0],
        [BOARD_IMAGE_SIZE - 1, 0],
        [BOARD_IMAGE_SIZE - 1, BOARD_IMAGE_SIZE - 1],
        [0, BOARD_IMAGE_SIZE - 1]
    ], dtype="float32")
    matrix = cv2.getPerspectiveTransform(expanded_points, dst_points)

    # フレームに射影変換を適用
    transformed = cv2.warpPerspective(frame, matrix, (BOARD_IMAGE_SIZE, BOARD_IMAGE_SIZE))

    # エッジを検出
    edges = cv2.Canny(cv2.cvtColor(transformed, cv2.COLOR_BGR2GRAY), 100, 200)

    # Hough変換で直線を検出
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

    # マスクを作成して直線成分以外を除去
    line_mask = np.zeros_like(edges, dtype=np.uint8)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_mask, (x1, y1), (x2, y2), 255, 1)

    # 直線成分以外をマスク
    #edges = cv2.bitwise_and(edges, line_mask)

    # 直線を延長 (default_pointsで作られる四角形から+-20ピクセル以内に制限)
    if lines is not None:
        # default_pointsの外側20ピクセルの四角形を計算
        points = np.array(default_points, dtype=np.float32)
        expanded_points = points.copy()
        for i, point in enumerate(points):
            prev_point = points[i - 1]
            next_point = points[(i + 1) % len(points)]
            direction = np.mean([point - prev_point, point - next_point], axis=0)
            direction = direction / np.linalg.norm(direction) * (10 if i == 0 or i == 3 else 20)  # iが0か3のときは10ピクセル拡張
            expanded_points[i] += direction

        # 四角形のマスクを作成
        mask = np.zeros_like(edges, dtype=np.uint8)
        cv2.fillPoly(mask, [expanded_points.astype(np.int32)], 255)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx, dy = x2 - x1, y2 - y1
            length = np.sqrt(dx**2 + dy**2)
            if length > 0:
                # 延長する長さを設定
                extend_length = BOARD_IMAGE_SIZE
                dx = dx / length * extend_length
                dy = dy / length * extend_length
                x1_ext = min(BOARD_IMAGE_SIZE - 1, max(0, int(x1 - dx)))
                y1_ext = min(BOARD_IMAGE_SIZE - 1, max(0, int(y1 - dy)))
                x2_ext = min(BOARD_IMAGE_SIZE - 1, max(0, int(x2 + dx)))
                y2_ext = min(BOARD_IMAGE_SIZE - 1, max(0, int(y2 + dy)))

                # 延長した直線が四角形のマスク内にある場合のみ描画
                if mask[y1, x1] > 0 and mask[y2, x2] > 0:
                    cv2.line(edges, (x1_ext, y1_ext), (x2_ext, y2_ext), 255, 1)

    # エッジ画像からコーナーを検出
    corners = cv2.goodFeaturesToTrack(edges, maxCorners=100, qualityLevel=0.01, minDistance=10)
    corners = np.int0(corners)

    # 検出したコーナーを表示（デバッグ用）
    if DEBUG_IMSHOW:
        for corner in corners:
            x, y = corner.ravel()
            cv2.circle(edges, (x, y), 3, (255, 0, 0), -1)  # 青い円を描画

    # 検出したエッジを表示（デバッグ用）
    if DEBUG_IMSHOW:
        cv2.imshow('Edge+Corner', edges)

    # 射影変換前に戻すための逆射影変換行列を計算
    inverse_matrix = cv2.getPerspectiveTransform(dst_points, expanded_points)

    # 検出したコーナーを射影変換前の座標に戻す
    corners = cv2.perspectiveTransform(corners.reshape(-1, 1, 2).astype(np.float32), inverse_matrix).reshape(-1, 2)

    # cornersをフレームに描画
    for corner in corners:
        x, y = corner.ravel()
        cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)  # 青い円を描画
    

    # expanded_pointsをフレームに描画
    for point in expanded_points:
        cv2.circle(frame, tuple(point.astype(int)), 5, (0, 255, 0), -1)  # 緑の円を描画
    # default_pointsをフレームに描画
    for point in default_points:
        cv2.circle(frame, tuple(point), 5, (255, 0, 255), -1)  # 紫の円を描画

    # frameを表示（デバッグ用）
    if DEBUG_IMSHOW:
        cv2.imshow('Corner Frame', frame)

    # default_pointsの4点付近で最も近いコーナーを探す（+-5ピクセルに制限）
    adjusted_points = []
    for i, point in enumerate(default_points):
        nearby_corners = [corner for corner in corners if 
                          abs(corner.ravel()[0] - point[0]) <= 5 and 
                          abs(corner.ravel()[1] - point[1]) <= 5]
        if nearby_corners:
            closest_corner = min(nearby_corners, key=lambda corner: np.linalg.norm(np.array(point) - corner.ravel()))
            adjusted_points.append(tuple([round(elem) for elem in closest_corner.ravel()]))
        else:
            adjusted_points.append(tuple(point))  # 制限内にコーナーがない場合は元の点を使用

    return adjusted_points

def get_transformed_board():
    # 4点を取得
    #points = get_points_single()
    points = default_points
    '''
    points_arr = []
    for _ in range(5):
        points_arr.append(get_points_single())
    points = []
    for i in range(4):
        point_arr = [points_arr[j][i] for j in range(len(points_arr))]
        #print(i, point_arr)
        x_values = [point[0] for point in point_arr]
        y_values = [point[1] for point in point_arr]
        x_min, x_max = np.percentile(x_values, [10, 90])
        y_min, y_max = np.percentile(y_values, [10, 90])
        filtered_points = [(x, y) for x, y in point_arr if x_min <= x <= x_max and y_min <= y <= y_max]
        centroid = np.mean(filtered_points, axis=0).astype(int)
        points.append(tuple(centroid))
    #'''


    # フレームを取得
    ret, frame = cap.read()
    if not ret:
        print('Cannot receive a frame')
        return None

    # 4点で囲われた範囲をBOARD_IMAGE_SIZExBOARD_IMAGE_SIZEの正方形画像に変換
    dst_points = np.array([
        [0, 0],
        [BOARD_IMAGE_SIZE - 1, 0],
        [BOARD_IMAGE_SIZE - 1, BOARD_IMAGE_SIZE - 1],
        [0, BOARD_IMAGE_SIZE - 1]
    ], dtype="float32")
    # 射影変換行列を計算
    matrix = cv2.getPerspectiveTransform(np.array(points, dtype="float32"), dst_points)
    # フレームに射影変換を適用
    transformed = cv2.warpPerspective(frame, matrix, (BOARD_IMAGE_SIZE, BOARD_IMAGE_SIZE))

    # 画像の向きを変換
    transformed = cv2.rotate(transformed, cv2.ROTATE_90_COUNTERCLOCKWISE)
    transformed = cv2.flip(transformed, 1)


    # 変換された画像の明るさの平均値を127に調整
    mean_val = cv2.mean(cv2.cvtColor(transformed, cv2.COLOR_BGR2GRAY))[0]
    brightness = mean_val
    adjustment = 127 - brightness
    transformed = cv2.convertScaleAbs(transformed, alpha=1, beta=adjustment)

    # 変換された画像をぼやかす
    transformed = cv2.blur(transformed, (3, 3))

    # 取得した点をフレーム上に描画
    for i, point in enumerate(points):
        cv2.circle(frame, default_points[i], 5, (255, 0, 0), -1)  # 青い円を描画
        cv2.circle(frame, point, 3, (0, 0, 255), -1)  # 赤い円を描画
        cv2.putText(frame, f"{i+1}", (point[0] + 10, point[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)  # 点番号を描画

    # フレームを表示
    if DEBUG_IMSHOW:
        cv2.imshow('Camera', frame)
    
    return transformed
    

# Webカメラを初期化 (デフォルトのカメラはID 0)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print('Cannot open camera')
    exit()




def get_disc_places_single():
    transformed = get_transformed_board()

    # 変換された画像を表示（デバッグ用）
    if DEBUG_IMSHOW:
        cv2.imshow('Transformed', transformed)

    disc_places = recognize_disc_place(transformed)
    #if board_arr is not None:
    #    print("Board:", board_arr)

    # ウィンドウを更新
    if DEBUG_IMSHOW:
        cv2.waitKey(1)

    return disc_places





def get_board():
    disc_places_arr = []
    for _ in range(20):
        disc_places_arr.append(get_disc_places_single())
    
    centroid_coords = [[], []]
    for color in range(2):
        coords_arr = [disc_places_arr[i][color] for i in range(len(disc_places_arr))]

        coord_cluster = [[coords_arr[0][i]] for i in range(len(coords_arr[0]))]

        for coords in coords_arr[1:]:
            for coord in coords:
                added = False
                for cluster in coord_cluster:
                    if np.linalg.norm(np.array(cluster[0]) - np.array(coord)) < 10:  # 距離が10未満なら同じクラスターに追加
                        cluster.append(coord)
                        added = True
                        break
                if not added:
                    coord_cluster.append([coord])  # 新しいクラスターを作成
        for cluster in coord_cluster:
            if len(cluster) > 0:
                cluster = np.array(cluster)
                x_values = cluster[:, 0]
                y_values = cluster[:, 1]
                x_min, x_max = np.percentile(x_values, [20, 80])
                y_min, y_max = np.percentile(y_values, [20, 80])
                filtered_cluster = cluster[
                    (x_values >= x_min) & (x_values <= x_max) &
                    (y_values >= y_min) & (y_values <= y_max)
                ]
                if len(filtered_cluster) > 1:
                    centroid = np.mean(filtered_cluster, axis=0).astype(int)
                    x, y = centroid
                    centroid_coords[color].append([x, y])
    

    transformed = get_transformed_board()

    diameter = int(BOARD_IMAGE_SIZE / 8 * 0.2)
    board_arr = [[EMPTY for _ in range(HW)] for _ in range(HW)]
    disc_slip_mm = [[[0, 0] for _ in range(HW)] for _ in range(HW)]
    for y in range(HW):
        for x in range(HW):
            leftupper_x = BOARD_IMAGE_SIZE / HW * x
            leftupper_y = BOARD_IMAGE_SIZE / HW * y + DISC_HEIGHT_SHIFT_TOP * (HW - 1 - y) / (HW - 1) + DISC_HEIGHT_SHIFT_BOTTOM * y / (HW - 1)
            rightbottom_x = leftupper_x + BOARD_IMAGE_SIZE / HW
            rightbottom_y = BOARD_IMAGE_SIZE / HW * (y + 1) + DISC_HEIGHT_SHIFT_TOP * (HW - 1 - (y + 1)) / (HW - 1) + DISC_HEIGHT_SHIFT_BOTTOM * (y + 1) / (HW - 1)

            # 四角形の範囲を整数に変換
            leftupper_x = int(leftupper_x)
            leftupper_y = int(leftupper_y)
            rightbottom_x = int(rightbottom_x)
            rightbottom_y = int(rightbottom_y)

            # 四角形を描画
            cv2.rectangle(transformed, (leftupper_x, leftupper_y), (rightbottom_x, rightbottom_y), (0, 255, 0), 1)

            for color in range(2):
                for coord in centroid_coords[color]:
                    disc_x, disc_y = coord
                    if leftupper_x <= disc_x < rightbottom_x and leftupper_y <= disc_y < rightbottom_y:
                        cx = (leftupper_x + rightbottom_x) / 2
                        cy = (leftupper_y + rightbottom_y) / 2
                        disc_slip_mm[y][x][0] = min(30, round((disc_x - cx) / (leftupper_x - rightbottom_x) * CELL_SIZE_MM))
                        disc_slip_mm[y][x][1] = min(30, round(-(disc_y - cy) / (leftupper_y - rightbottom_y) * CELL_SIZE_MM))
                        if color == BLACK:
                            board_arr[y][x] = BLACK
                            cv2.circle(transformed, (disc_x, disc_y), diameter // 2, (0, 0, 255), 2)
                        else:
                            board_arr[y][x] = WHITE
                            cv2.circle(transformed, (disc_x, disc_y), diameter // 2, (255, 0, 0), 2)
                        break

    # 変換された画像を表示（デバッグ用）
    if DEBUG_IMSHOW:
        cv2.imshow('Recognized', transformed)

    # オセロの盤面画像を作成
    board_image = np.zeros((BOARD_IMAGE_SIZE, BOARD_IMAGE_SIZE, 3), dtype=np.uint8)
    # 背景を緑に設定
    board_image[:] = (0, 128, 0)
    cell_size = BOARD_IMAGE_SIZE // HW
    radius = round(cell_size * 0.82 / 2)
    # グリッド線を描画
    for i in range(1, HW):
        cv2.line(board_image, (0, i * cell_size), (BOARD_IMAGE_SIZE, i * cell_size), (0, 0, 0), 1)
        cv2.line(board_image, (i * cell_size, 0), (i * cell_size, BOARD_IMAGE_SIZE), (0, 0, 0), 1)
    # 石を描画
    for y in range(HW):
        for x in range(HW):
            center_x = round(x * cell_size + cell_size // 2 - disc_slip_mm[y][x][0] / CELL_SIZE_MM * cell_size)
            center_y = round(y * cell_size + cell_size // 2 + disc_slip_mm[y][x][1] / CELL_SIZE_MM * cell_size)
            if board_arr[y][x] == BLACK:
                cv2.circle(board_image, (center_x, center_y), radius, (0, 0, 0), -1)
            elif board_arr[y][x] == WHITE:
                cv2.circle(board_image, (center_x, center_y), radius, (255, 255, 255), -1)
    # 盤面画像を表示
    if DEBUG_IMSHOW:
        cv2.imshow('Othello Board', board_image)
    
    # ウィンドウを更新
    if DEBUG_IMSHOW:
        cv2.waitKey(1)

    for arr in disc_slip_mm:
        print(arr)
    print('')
    #print(board_arr)
    return board_arr, disc_slip_mm
    

        
    

def cleanup_camera():
    cap.release()
    cv2.destroyAllWindows()
