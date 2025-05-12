import cv2
import numpy as np
from othello import BLACK, WHITE, EMPTY, HW, HW2

# グローバル変数でクリックした座標を保存
#points = []
points = [(185, 226), (94, 365), (505, 368), (410, 226)]

'''
def mouse_callback(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:  # 左クリック時
        points.append((x, y))
        print(f"Point {len(points)}: {x}, {y}")
        if len(points) == 4:
            print("4つの点を取得しました:", points)
'''
            
def analyze_cell_colors_and_display(warped, divisions):
    """セル内のピクセルを黒、白、緑に分類し、割合を計算し、グリッドに表示"""
    step = warped.shape[0] // divisions
    color_grid = np.zeros((warped.shape[0], warped.shape[1], 3), dtype=np.uint8)  # グリッド用の画像

    board_arr = [[EMPTY for _ in range(HW2)] for _ in range(HW2)]

    for i in range(divisions):
        for j in range(divisions):
            # 各セルの領域を取得
            cell = warped[i * step:(i + 1) * step, j * step:(j + 1) * step]

            # 総ピクセル数
            total_pixels = cell.shape[0] * cell.shape[1]

            # 各ピクセルを黒、白、緑に分類
            black = np.sum(np.all(cell < [150, 140, 150], axis=2))  # 黒: RGBが100未満
            white = np.sum(np.all(cell > [200, 200, 200], axis=2))  # 白: RGBが200以上
            #green = np.sum((cell[:, :, 1] > 100) & (cell[:, :, 0] < 100) & (cell[:, :, 2] < 100))  # 緑: Gが高く、RとBが低い
            #green = total_pixels - black - white

            # 割合を計算
            black_ratio = (black / total_pixels) * 100
            white_ratio = (white / total_pixels) * 100
            #green_ratio = (green / total_pixels) * 100

            #print(i, j, black_ratio, white_ratio)
    
            circle_color = (-1, -1, -1)
            if black_ratio > 30.0:
                circle_color = (0, 0, 0) # 黒
                board_arr[HW - 1 - j][HW - 1 - i] = BLACK
            elif white_ratio > 30.0:
                circle_color = (255, 255, 255) # 白
                board_arr[HW - 1 - j][HW - 1 - i] = WHITE
            #else:
            #    color = (0, 255, 0)  # 緑

            # グリッド用画像にセルの色を塗る
            cv2.rectangle(color_grid, (i * step, j * step), ((i + 1) * step, (j + 1) * step), (0, 255, 50), -1)
            cv2.rectangle(color_grid, (i * step, j * step), ((i + 1) * step, (j + 1) * step), (0, 0, 0), 1)
            if circle_color != (-1, -1, -1):
                cv2.circle(color_grid, (i * step + step // 2, j * step + step // 2), round(step / 2.5), circle_color, -1)

    # 別画面にグリッドを表示
    #cv2.imshow("Board", color_grid)

    return board_arr

def draw_grid_and_analyze(frame, points):
    """正方形を8×8に分割して描画し、各セルの色を分析"""
    if len(points) == 4:
        # 射影変換のための正方形の座標
        square_size = 400  # 正方形のサイズ（ピクセル）
        dst_points = np.array([
            [0, 0],
            [square_size - 1, 0],
            [square_size - 1, square_size - 1],
            [0, square_size - 1]
        ], dtype="float32")

        # 射影変換行列を計算
        matrix = cv2.getPerspectiveTransform(np.array(points, dtype="float32"), dst_points)

        # フレームに射影変換を適用
        warped = cv2.warpPerspective(frame, matrix, (square_size, square_size))

        # グリッドを描画
        step = square_size // HW
        for i in range(1, HW):
            # 水平線
            cv2.line(warped, (0, i * step), (square_size, i * step), (0, 255, 0), 1)
            # 垂直線
            cv2.line(warped, (i * step, 0), (i * step, square_size), (0, 255, 0), 1)

        # セル内の色を分析して別画面に表示
        return analyze_cell_colors_and_display(warped, HW)

        # 元のフレームに逆射影変換でグリッドを戻す
        #inverse_matrix = cv2.getPerspectiveTransform(dst_points, np.array(points, dtype="float32"))
        #grid_overlay = cv2.warpPerspective(warped, inverse_matrix, (frame.shape[1], frame.shape[0]))
        #frame[:] = cv2.addWeighted(frame, 1, grid_overlay, 0.2, 0)
    return None

def recognize_board(transformed):
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

    #cv2.imshow('color_mask', color_mask)

    # マスクを反転
    disc_mask = cv2.bitwise_not(color_mask)

    # マスクを適用して抽出
    result = cv2.bitwise_and(transformed, transformed, mask=disc_mask)

    # デバッグ用にマスクと結果を表示
    cv2.imshow('disc_mask', disc_mask)
    cv2.imshow('Filtered', result)

    # 距離変換を適用して、mask外からの距離を計算
    distance_transform = cv2.distanceTransform(disc_mask, cv2.DIST_L2, 5)

    # 距離を正規化して0-255の範囲にスケール
    normalized_distance = cv2.normalize(distance_transform, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # 距離画像を表示
    cv2.imshow('Distance Transform', normalized_distance)

    # 各連結部分の中で、距離の最大値の1/2以上のピクセルのみを計算
    # disc_maskを連結部分ごとに分けてループ
    num_labels, labels = cv2.connectedComponents(disc_mask)
    filtered_distance = np.zeros_like(normalized_distance, dtype=np.uint8)
    for label in range(1, num_labels):  # ラベル0は背景なのでスキップ
        # 各連結部分のマスクを作成
        region_mask = ((labels == label) * 255).astype(np.uint8)
        n_pixcels = np.sum(region_mask) // 255
        if n_pixcels < 256 * 256 // (HW2 * 10):
            continue
        print(f"Label {label}: {n_pixcels} pixels")

        # 距離画像にマスクを適用して最大値を取得
        max_distance = cv2.minMaxLoc(distance_transform, mask=region_mask)[1]
        threshold = max_distance * 0.6

        # 最大値の1/2以上のピクセルを保持
        filtered_distance = cv2.bitwise_or(
            filtered_distance,
            cv2.inRange(distance_transform, threshold, max_distance) & region_mask
        )
    # デバッグ用にフィルタリングされた距離画像を表示
    cv2.imshow('Filtered Distance (Max/2)', filtered_distance)

    # グレースケール化
    gray_transformed = cv2.cvtColor(transformed, cv2.COLOR_BGR2GRAY)
    # adaptiveThresholdで2値化
    binary_transformed = cv2.adaptiveThreshold(
        gray_transformed, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )
    # デバッグ用に2値化された画像を表示
    cv2.imshow('Binary Transformed', binary_transformed)


    
    # 各連結部分の重心点から円を描き、石の色を判定する
    num_labels_discs, labels_discs = cv2.connectedComponents(filtered_distance)
    diameter = 256 / 8 * 0.8
    
    board_arr = [[EMPTY for _ in range(HW)] for _ in range(HW)]
    for y in range(HW):
        for x in range(HW):
            leftupper_x = 256 / HW * x
            leftupper_y = 256 / HW * y
            rightbottom_x = leftupper_x + 256 / HW
            rightbottom_y = leftupper_y + 256 / HW

            # 四角形の範囲を整数に変換
            leftupper_x = int(leftupper_x)
            leftupper_y = int(leftupper_y)
            rightbottom_x = int(rightbottom_x)
            rightbottom_y = int(rightbottom_y)

            # 四角形の中のマスクを抽出
            region_mask = filtered_distance[leftupper_y:rightbottom_y, leftupper_x:rightbottom_x]

            # マスクが存在するかを判定
            if np.any(region_mask > 0):
                for label in range(1, num_labels_discs):  # ラベル0は背景なのでスキップ
                    # 各連結部分のマスクを作成
                    region_mask = (labels_discs == label).astype(np.uint8)

                    # 重心を計算
                    moments = cv2.moments(region_mask)
                    if moments["m00"] != 0:
                        cx = int(moments["m10"] / moments["m00"])
                        cy = int(moments["m01"] / moments["m00"])
                    else:
                        continue
                    if leftupper_x <= cx <= rightbottom_x and leftupper_y <= cy <= rightbottom_y:
                        # 円の中のピクセルを抽出するためのマスクを作成
                        circle_mask = np.zeros_like(binary_transformed, dtype=np.uint8)
                        cv2.circle(circle_mask, (cx, cy), int(diameter / 2), 255, -1)
                        # マスクを適用して円内のピクセルを抽出
                        circle_pixels = cv2.bitwise_and(binary_transformed, binary_transformed, mask=circle_mask)
                        # 白と黒のピクセル数を数える
                        white_pixels = cv2.countNonZero(circle_pixels)
                        black_pixels = np.sum(circle_mask // 255) - white_pixels
                        print(f"Label {label}: White pixels = {white_pixels}, Black pixels = {black_pixels}")
                        if black_pixels > white_pixels * 0.4:
                            board_arr[y][x] = BLACK
                            cv2.circle(transformed, (cx, cy), int(diameter / 2), (0, 0, 0), 2)
                        else:
                            board_arr[y][x] = WHITE
                            cv2.circle(transformed, (cx, cy), int(diameter / 2), (255, 255, 255), 2)
                        break
            
    # デバッグ用に円を描画した画像を表示
    cv2.imshow('Circles on Transformed', transformed)


    # オセロの盤面画像を作成
    board_image = np.zeros((256, 256, 3), dtype=np.uint8)
    # 背景を緑に設定
    board_image[:] = (0, 128, 0)
    cell_size = 256 // HW
    # グリッド線を描画
    for i in range(1, HW):
        cv2.line(board_image, (0, i * cell_size), (256, i * cell_size), (0, 0, 0), 1)
        cv2.line(board_image, (i * cell_size, 0), (i * cell_size, 256), (0, 0, 0), 1)
    # 石を描画
    for y in range(HW):
        for x in range(HW):
            center = (x * cell_size + cell_size // 2, y * cell_size + cell_size // 2)
            radius = cell_size // 3
            if board_arr[y][x] == BLACK:
                cv2.circle(board_image, center, radius, (0, 0, 0), -1)
            elif board_arr[y][x] == WHITE:
                cv2.circle(board_image, center, radius, (255, 255, 255), -1)
    # 盤面画像を表示
    cv2.imshow('Othello Board', board_image)
    


    print(board_arr)
    return board_arr

    

# Webカメラを初期化 (デフォルトのカメラはID 0)
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print('Cannot open camera')
    exit()

# ウィンドウを作成し、マウスコールバックを設定
#cv2.namedWindow('Camera')
#cv2.setMouseCallback('Camera', mouse_callback)

def get_board():
    # フレームを取得
    ret, frame = cap.read()

    # フレームのコントラストを上げる
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    lab = cv2.merge((l, a, b))
    frame = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    if not ret:
        print('Cannot receive a frame')
        return None

    # 4点で囲われた範囲を256x256の正方形画像に変換
    square_size = 256  # 正方形のサイズ（ピクセル）
    dst_points = np.array([
        [0, 0],
        [square_size - 1, 0],
        [square_size - 1, square_size - 1],
        [0, square_size - 1]
    ], dtype="float32")
    # 射影変換行列を計算
    matrix = cv2.getPerspectiveTransform(np.array(points, dtype="float32"), dst_points)
    # フレームに射影変換を適用
    transformed = cv2.warpPerspective(frame, matrix, (square_size, square_size))

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

    # 変換された画像を表示（デバッグ用）
    cv2.imshow('Transformed', transformed)

    board = recognize_board(transformed)
    board = None
    if board is not None:
        print("Board:", board)


    # 取得した点をフレーム上に描画
    for i, point in enumerate(points):
        cv2.circle(frame, point, 5, (0, 0, 255), -1)  # 赤い円を描画
        cv2.putText(frame, f"{i+1}", (point[0] + 10, point[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)  # 点番号を描画

    # フレームを表示
    cv2.imshow('Camera', frame)

    # ウィンドウを更新
    cv2.waitKey(1)

    return board


def cleanup_camera():
    cap.release()
    cv2.destroyAllWindows()
