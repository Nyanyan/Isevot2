import cv2
import numpy as np

BLACK = 0
WHITE = 1
EMPTY = 2

# グローバル変数でクリックした座標を保存
#points = []
points = [(190, 226), (98, 365), (512, 368), (416, 226)]

def mouse_callback(event, x, y, flags, param):
    """マウスイベントのコールバック関数"""
    global points
    if event == cv2.EVENT_LBUTTONDOWN:  # 左クリック時
        points.append((x, y))
        print(f"Point {len(points)}: {x}, {y}")
        if len(points) == 4:
            print("4つの点を取得しました:", points)

def analyze_cell_colors_and_display(warped, divisions):
    """セル内のピクセルを黒、白、緑に分類し、割合を計算し、グリッドに表示"""
    step = warped.shape[0] // divisions
    color_grid = np.zeros((warped.shape[0], warped.shape[1], 3), dtype=np.uint8)  # グリッド用の画像

    res = []

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
                res.append(BLACK)
            elif white_ratio > 30.0:
                circle_color = (255, 255, 255) # 白
                res.append(WHITE)
            else:
                res.append(EMPTY)
            #else:
            #    color = (0, 255, 0)  # 緑

            # グリッド用画像にセルの色を塗る
            cv2.rectangle(color_grid, (i * step, j * step), ((i + 1) * step, (j + 1) * step), (0, 255, 50), -1)
            cv2.rectangle(color_grid, (i * step, j * step), ((i + 1) * step, (j + 1) * step), (0, 0, 0), 1)
            if circle_color != (-1, -1, -1):
                cv2.circle(color_grid, (i * step + step // 2, j * step + step // 2), round(step / 2.5), circle_color, -1)

    # 別画面にグリッドを表示
    cv2.imshow("Board", color_grid)

    return res

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
        divisions = 8
        step = square_size // divisions
        for i in range(1, divisions):
            # 水平線
            cv2.line(warped, (0, i * step), (square_size, i * step), (0, 255, 0), 1)
            # 垂直線
            cv2.line(warped, (i * step, 0), (i * step, square_size), (0, 255, 0), 1)

        # セル内の色を分析して別画面に表示
        return analyze_cell_colors_and_display(warped, divisions)

        # 元のフレームに逆射影変換でグリッドを戻す
        #inverse_matrix = cv2.getPerspectiveTransform(dst_points, np.array(points, dtype="float32"))
        #grid_overlay = cv2.warpPerspective(warped, inverse_matrix, (frame.shape[1], frame.shape[0]))
        #frame[:] = cv2.addWeighted(frame, 1, grid_overlay, 0.2, 0)
    return None

def capture_webcam():
    # Webカメラを初期化 (デフォルトのカメラはID 0)
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print('Cannot open camera')
        return

    # ウィンドウを作成し、マウスコールバックを設定
    cv2.namedWindow('Camera')
    cv2.setMouseCallback('Camera', mouse_callback)

    while True:
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
            break

        # 取得した点をフレーム上に描画
        for i, point in enumerate(points):
            cv2.circle(frame, point, 5, (0, 0, 255), -1)  # 赤い円を描画
            cv2.putText(frame, f"{i+1}", (point[0] + 10, point[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)  # 点番号を描画

        # 4点が取得された場合、正方形を8×8に分割して描画し、セルの色を分析
        if len(points) == 4:
            # 4点で囲われた範囲の明るさの平均値を127に調整
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask, [np.array(points, dtype=np.int32)], 255)
            mean_val = cv2.mean(frame, mask=mask)[:3]
            brightness = np.mean(mean_val)
            adjustment = 127 - brightness
            frame = cv2.convertScaleAbs(frame, alpha=1, beta=adjustment)

            board = draw_grid_and_analyze(frame, points)
            if board is not None:
                print("Board:", board)

        # フレームを表示
        cv2.imshow('Camera', frame)

        # 'q'キーで終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # リソースを解放
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_webcam()