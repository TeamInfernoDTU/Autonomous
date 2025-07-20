#!/usr/bin/env python3
import numpy as np
import cv2


# distance = 0

# rospy.init_node("driving")
# pub = rospy.Publisher("/cmd_vel", Twist,queue_size=10)


def preprocess(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, img_thres = cv2.threshold(img_gray, 70, 255, cv2.THRESH_TOZERO)
    img_blur = cv2.bilateralFilter(img_thres, 5, 75, 75)
    img_canny = cv2.Canny(img_blur, 50, 50)
    kernel = np.ones((3, 3))
    img_dilate = cv2.dilate(img_canny, kernel, iterations=1)
    img_erode = cv2.erode(img_dilate, kernel, iterations=1)
    return img_erode


def find_tip(points, convex_hull):
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)
    for i in range(2):
        j = indices[i] + 2
        if np.all(points[j % length] == points[indices[i - 1] - 2]):
            return tuple(points[j % length]), j % length
    return None, None


def find_tail_rect(points, convex_hull):
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)
    direction = None
    for i in range(2):
        j = (indices[i] + 2) % length
        if np.all(points[j] == points[indices[i - 1] - 2]):
            sides = []  # length of sides of the tail rectangle
            prev_pt = points[(indices[i - 1] + 1) % length]
            for pt in (
                points[indices[i] - 1],
                points[indices[i]],
                points[indices[i - 1]],
                points[(indices[i - 1] + 1) % length],
            ):
                sides.append(np.linalg.norm(pt - prev_pt))
                prev_pt = pt

            if (
                abs(sides[0] - sides[2]) / float(max(sides[2], sides[0])) < 0.5
                and abs(sides[1] - sides[3]) / float(sides[1]) < 0.15
            ):
                
                if points[indices[i] - 1][0] < points[indices[i]][0]:
               
                    direction = 1  # TODO : Add respective rect pts in order
                else:
   
                    direction = 0
                if points[indices[i - 1]][1] < points[indices[i]][1]:
      
                    return (
                        np.array(
                            (
                                points[indices[i] - 1],
                                points[indices[i]],
                                points[indices[i - 1]],
                                points[(indices[i - 1] + 1) % length],
                            )
                        ),
                        direction,
                    )
                return (
                    np.array(
                        (
                            points[(indices[i - 1] + 1) % length],
                            points[indices[i - 1]],
                            points[indices[i]],
                            points[indices[i] - 1],
                        )
                    ),
                    direction,
                )
    return None, None


def correct_corners(points, corners):
    new_points = []
    for n, pt in enumerate(points):
        err = (
            5 if not n in [3, 4] else 0
        )  
        if err == 0:
            new_points.append(pt)
            continue
        new_pt = corners[np.argmin([np.linalg.norm(corner - pt) for corner in corners])]
       
        new_pt = new_pt if np.linalg.norm(new_pt - pt) < err else pt
        new_points.append(new_pt)
    return np.array(new_points)

def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1, 2)
    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)
    # draw pillars in blue color
    for i, j in zip(range(4), range(4, 8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)
    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)
    return img


def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result, rot_mat


def get_arrow_arr(img, debug=True):
    if debug:
        cv2.imshow("Image", img)
        cv2.waitKey(0)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, img_thres = cv2.threshold(img_gray, 120, 255, cv2.THRESH_OTSU)
    img_blur = cv2.GaussianBlur(img_thres, (5, 5), 1)
    img = cv2.bilateralFilter(img_thres, 5, 75, 75)
    contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    if debug:
        cv2.imshow("Image", img)
        cv2.waitKey(0)

    for cnt in contours:
        if cv2.contourArea(cnt) < 200:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
        hull = cv2.convexHull(approx, returnPoints=False)
        sides = len(hull)

        if (sides == 5 or sides == 4) and sides + 2 == len(approx):
            if debug:
                img_tmp = img.copy()
         
                cv2.drawContours(img_tmp, [approx], -1, (100), 1)
                #cv2.imshow("contour", img_tmp)
                cv2.waitKey(0)
            arrow_tip, tip_idx = find_tip(approx[:, 0, :], hull.squeeze())
            if arrow_tip is None:
                continue
            points = np.roll(approx[:, 0, :], -tip_idx)
            if points[1][1] < arrow_tip[1]:
                points = np.flipud(np.roll(points, -1, axis=0))  # for uniformity
         
            img_inv = cv2.bitwise_not(img)
            h, w = img.shape[:2]
            mask1 = np.zeros((h + 2, w + 2), np.uint8)
            ret, _, mask1, _ = cv2.floodFill(
                cv2.erode(img.copy(), np.ones((3, 3), np.uint8)),
                mask1,
                tuple(np.uint8(np.average(points, axis=0))),
                255,
                flags=cv2.FLOODFILL_MASK_ONLY,
            )  
            mask1 = mask1[1:-1, 1:-1]
            mask_inv = cv2.bitwise_not(mask1)
            masked_img = cv2.bitwise_and(img, img, mask=mask1)
          

            corners = cv2.goodFeaturesToTrack(img, 25, 0.0001, 10, mask=mask1).reshape(
                -1, 2
            )
            corners2 = [[-1], [-1], [-1], [-1]]
            max_vals = [-1e5, -1e5, -1e5, -1e5]  # x+y, x-y, y-x, -y-x
            lim = int(np.floor(2 * np.linalg.norm(points[3] - points[4]) / 3))
            lim = min(lim, 10)
            direction = (points[0] - points[1])[0] > 0  # left = 0, right = 1
            for i in range(-lim, lim):
                for j in range(-lim, lim):
                    x, y = points[3] + [i, j]
                    if img[y, x] == 255 or mask1[y, x] == 0:
                        continue
                    for k, fn in enumerate(
                        [
                            lambda x, y: x + y,
                            lambda x, y: x - y,
                            lambda x, y: y - x,
                            lambda x, y: -x - y,
                        ]
                    ):
                        if fn(x, y) > max_vals[k]:
                            max_vals[k] = fn(x, y)
                            corners2[k] = x, y
            
            points[3] = (
                corners2[2] if direction else corners2[0]
            )  
            corners2 = [[-1], [-1], [-1], [-1]]
            max_vals = [-1e5, -1e5, -1e5, -1e5]  # x+y, x-y, y-x, -y-x
            for i in range(-lim, lim):
                for j in range(-lim, lim):
                    x, y = points[4] + [i, j]

                    if img[y, x] == 255 or mask1[y, x] == 0:
                        continue
                    for k, fn in enumerate(
                        [
                            lambda x, y: x + y,
                            lambda x, y: x - y,
                            lambda x, y: y - x,
                            lambda x, y: -x - y,
                        ]
                    ):
                        if fn(x, y) > max_vals[k]:
                            max_vals[k] = fn(x, y)
                            corners2[k] = x, y

            points[4] = (
                corners2[3] if direction else corners2[1]
            )  
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            corners = cv2.cornerSubPix(
                img, np.float32(corners), (3, 3), (-1, -1), criteria
            )
      
            corners = np.uint8(corners)
            if debug:
                img_tmp = img.copy()
                for corner in corners:
                    cv2.circle(img_tmp, tuple(corner), 3, (125), cv2.FILLED)
                cv2.imshow("corners", img_tmp)
                cv2.waitKey(0)
            points = correct_corners(points, corners)
          
            if debug:
                img_tmp = img.copy()
                for n, i in enumerate(points):
                    cv2.circle(img_tmp, tuple(i), 3, (125), cv2.FILLED)
                cv2.imshow(str(n) + "th point", img_tmp)
                cv2.waitKey(0)

            return points


def arrow_detect(img, far=True):

    orig_img = img.copy()
    found = False
    theta = None
    orient = None
    direction = None
    bounding_box = None
    contours, _ = cv2.findContours(
        preprocess(img), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )[-2:]
 
    for cnt in contours:
        if cv2.contourArea(cnt) < 300:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
        hull = cv2.convexHull(approx, returnPoints=False)
        sides = len(hull)

        if (sides == 5 or sides == 4) and sides + 2 == len(approx):
            arrow_tip, _ = find_tip(approx[:, 0, :], hull.squeeze())
            rect, dirct = find_tail_rect(approx[:, 0, :], hull.squeeze())
            if arrow_tip and rect is not None:
        
                arrow_tail = tuple(np.average([rect[0], rect[3]], axis=0).astype(int))
                if (
                    arrow_tail[0] - arrow_tip[0] == 0
                ):  # to avoid division by 0 in next step
                    continue
               
                # Check that tan of angle of the arrow in the image from horizontal is less than 0.2(we are expecting nearly horizontal arrows)(atan(0.2) = 11.31)
                if (
                    abs(
                        float(arrow_tail[1] - arrow_tip[1])
                        / (arrow_tail[0] - arrow_tip[0])
                    )
                    > 0.2
                ):
                    continue  # Discard it, not a horizontal arrow
                
                theta = (
                    -(
                        np.average([arrow_tail[0], arrow_tip[0]]) / (np.shape(img)[0])
                        - 0.5
                    )
                    * 45
                    * 2
                )  # linear estimate, assuming camera horizontal range from -45 to 45
                direction = dirct  # TODO multiple arrow case
                found = True
                bounding_box = cv2.boundingRect(cnt)
                x,y,w,h = bounding_box
                pixel_width = w
                global distance 
                distance = ((29.6)*833.44709)/pixel_width
           
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
                cv2.drawContours(img, [approx], -1, (0, 150, 155), 2)
                cv2.circle(img, arrow_tip, 3, (0, 0, 255), cv2.FILLED)
  

    if (
        direction is not None and far == False
    ):  # TODO: Improve upon this naive orientation
        new_img = orig_img[
            int(bounding_box[1]) - 10 : int(bounding_box[1] + bounding_box[3] + 10),
            int(bounding_box[0]) - 10 : int(bounding_box[0] + bounding_box[2] + 10),
        ]
        train_pts = get_arrow_arr(new_img, False)
  
        new_train_pts = []
        for i, pt in enumerate(train_pts):
            new_pt = [
                pt[0] + int(bounding_box[0]) - 10,
                pt[1] + int(bounding_box[1]) - 10,
            ]
            new_train_pts.append(new_pt)
        train_pts = np.array(new_train_pts)
    
        new_img = orig_img.copy()
        query_pts = np.array(
            [
                [663, 197],
                [476, 326],
                [474, 234],
                [31, 232],
                [30, 162],
                [473, 162],
                [476, 69],
            ]
        )  # get_arrow_arr(template, False)
        if train_pts is None:
            print("not found in close up")
            return False, None, None, None, img
        matrix, mask = cv2.findHomography(query_pts, train_pts, 0, 5.0)
  
        mat_inv = np.linalg.inv(matrix)
     
        h, w, d = 416, 686, 3  # template.shape
        pts = np.float32(
            [[10, 10], [10, h - 10], [w - 10, h - 10], [w - 10, 10]]
        ).reshape(
            -1, 1, 2
        )  
        dst = cv2.perspectiveTransform(pts, matrix)
        homography = cv2.polylines(new_img, [np.int32(dst)], True, (255, 0, 0), 3)
        cam_mat = np.array([[480.0, 0, 400], [0, 465.0, 400], [0, 0, 1]])
        axis = (
            np.float32(
                [
                    [0, 0, 0],
                    [0, 3, 0],
                    [3, 3, 0],
                    [3, 0, 0],
                    [0, 0, -3],
                    [0, 3, -3],
                    [3, 3, -3],
                    [3, 0, -3],
                ]
            )
            * 50
        )
      
        axes_img = new_img.copy()
        ret, rvecs, tvecs = cv2.solvePnP(
            np.c_[query_pts, np.zeros(7)].astype(np.float32),
            train_pts.astype(np.float32),
            cam_mat,
            0,
        )
  
        r_mtx, _ = cv2.Rodrigues(rvecs)
        pm = cam_mat.dot(np.c_[r_mtx, tvecs])
        ea = cv2.decomposeProjectionMatrix(pm)[-1]
 
        imgpts, jac = cv2.projectPoints(
            axis, rvecs, tvecs, cam_mat.astype(np.float32), 0
        )
        axes_img = draw(axes_img, train_pts[2:], imgpts)
        img = axes_img
     
        orient = ea[1]
     
    if far == True:
        orient = 0
    if direction is not None:
        if direction == 1:  # Right
            orient = -90 - orient
        elif direction == 0:  # Left
            orient = 90 + orient
        else:
            print("error: direction not found and not None, " + str(direction))
            found = False
    # return found, theta, orient, direction, img, distance
    return direction
