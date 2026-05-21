import cv2
import numpy as np

cap = cv2.VideoCapture(0)

orb = cv2.ORB_create(nfeatures=1500)
bf = cv2.BFMatcher(cv2.NORM_HAMMING)

# Rough camera matrix. Later we should replace this with real calibration.
W, H = 640, 480
F = 500
K = np.array([
    [F, 0, W // 2],
    [0, F, H // 2],
    [0, 0, 1]
], dtype=np.float32)

prev_gray = None
prev_kp = None
prev_des = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed")
        break

    frame = cv2.resize(frame, (W, H))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    kp, des = orb.detectAndCompute(gray, None)

    if prev_gray is not None and prev_des is not None and des is not None:
        matches = bf.knnMatch(prev_des, des, k=2)

        good = []
        for pair in matches:
            if len(pair) == 2:
                m, n = pair
                if m.distance < 0.75 * n.distance:
                    good.append(m)

        if len(good) > 20:
            pts1 = np.float32([prev_kp[m.queryIdx].pt for m in good])
            pts2 = np.float32([kp[m.trainIdx].pt for m in good])

            E, mask = cv2.findEssentialMat(
                pts2, pts1, K,
                method=cv2.RANSAC,
                prob=0.999,
                threshold=1.0
            )

            if E is not None:
                _, R, t, mask_pose = cv2.recoverPose(E, pts2, pts1, K)

                x_motion = t[0][0]
                z_motion = t[2][0]

                print(f"matches={len(good)}  x={x_motion:.3f}  forward={z_motion:.3f}")

        match_img = cv2.drawMatches(
            prev_gray, prev_kp,
            gray, kp,
            good[:50],
            None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )

        cv2.imshow("SLAM feature matches", match_img)

    prev_gray = gray
    prev_kp = kp
    prev_des = des

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()