import cv2
import numpy as np
from othello import BLACK, WHITE, EMPTY, HW, HW2

BOARD_IMAGE_SIZE = 256
CELL_SIZE_MM = 29.0
DISC_HEIGHT_SHIFT_TOP = 5 #4.42 / (CELL_SIZE_MM * HW) * BOARD_IMAGE_SIZE
DISC_HEIGHT_SHIFT_BOTTOM = 13 #5.53 / (CELL_SIZE_MM * HW) * BOARD_IMAGE_SIZE

DEBUG_IMSHOW = True

# グローバル変数でクリックした座標を保存
default_points = [(185, 239), (78, 389), (510, 392), (410, 242)]

def recognize_disc_place(transformed):
    # 画像をHSVに変換
    hsv = cv2.cvtColor(transformed, cv2.COLOR_BGR2HSV)

    # 緑を抽出（明るさ120以上） 参考: https://qiita.com/tanaka-a/items/fe6b95ae922b684021cc
    hsv = cv2.cvtColor(transformed, cv2.COLOR_BGR2HSV)
    lower = np.array([45, 50, 120])
    upper = np.array([90, 255, 255])
    green1 = cv2.inRange(hsv, lower, upper)
    lower = np.array([45, 64, 120])
    upper = np.array([90, 255, 255])
    green2 = cv2.inRange(hsv, lower, upper)
    color_mask = cv2.bitwise_or(green1, green2)
    # マスクを反転
    disc_mask = cv2.bitwise_not(color_mask)

    # デバッグ用にマスクと結果を表示
    if DEBUG_IMSHOW:
        cv2.imshow('disc_mask', disc_mask)

    # 画像の周囲5ピクセルだけのマスクを作成
    border_mask = np.zeros_like(disc_mask, dtype=np.uint8)
    cv2.rectangle(border_mask, (0, 0), (disc_mask.shape[1], 5), 255, -1)  # 上辺
    cv2.rectangle(border_mask, (0, disc_mask.shape[0] - 5), (disc_mask.shape[1], disc_mask.shape[0]), 255, -1)  # 下辺
    cv2.rectangle(border_mask, (0, 0), (5, disc_mask.shape[0]), 255, -1)  # 左辺
    cv2.rectangle(border_mask, (disc_mask.shape[1] - 5, 0), (disc_mask.shape[1], disc_mask.shape[0]), 255, -1)  # 右辺

    if DEBUG_IMSHOW:
        cv2.imshow('Border Mask', border_mask)

    # 画像の周囲5ピクセル内に収まる連結部分を削除
    contours, _ = cv2.findContours(disc_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        mask = np.zeros_like(disc_mask, dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        intersection = cv2.bitwise_and(mask, border_mask)
        if np.array_equal(intersection, mask):
            cv2.drawContours(disc_mask, [contour], -1, 0, -1)

    # デバッグ用にマスクと結果を表示
    if DEBUG_IMSHOW:
        cv2.imshow('disc_mask_updated', disc_mask)


    # マスクを適用して抽出
    #discs = cv2.bitwise_and(transformed, transformed, mask=disc_mask)

    # transformedの明るさが190以上のピクセルを抽出
    brightness_mask = cv2.inRange(cv2.cvtColor(transformed, cv2.COLOR_BGR2GRAY), 190, 255)
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

def get_transformed_board():
    # 4点を取得
    points = default_points


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


    # 画像の緑成分（HSVのH:45-90, S:50-, V:30-）の明るさの平均値を180に調整
    hsv = cv2.cvtColor(transformed, cv2.COLOR_BGR2HSV)
    lower = np.array([45, 50, 30])
    upper = np.array([90, 255, 255])
    green_mask = cv2.inRange(hsv, lower, upper)
    v_channel = hsv[:, :, 2]
    green_v = v_channel[green_mask > 0]
    if len(green_v) > 0:
        mean_green_v = np.mean(green_v)
        adjustment = 180 - mean_green_v
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
    #if DEBUG_IMSHOW:
    #    cv2.imshow('Camera', frame)
    
    return transformed
    

# Webカメラを初期化 (デフォルトのカメラはID 0)
cap = cv2.VideoCapture(1)
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
