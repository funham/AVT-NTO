import cfg
import cv2
import numpy as np


class PerspectiveTransformation:
    """ This a class for transforming image between front view and top view """

    def __init__(self, in_size: tuple, out_size: tuple,
                 toffset,
                 boffset,
                 margin,  
                 height,   
                 bwidth,   
                 twidth,   
                 wscale):
        
        """Init PerspectiveTransformation.
        
        Arguments:
        
        in_size:  size of an input image
        out_size: size of an transformed image
        toffset:  offset of the transformation, from -1.0 to 1.0, where 0.0 is no offset
        boffset:  offset of the transformation, from -1.0 to 1.0, where 0.0 is no offset
        margin:   bottom margin of the transformation
        height:   height of the perspective
        bwidth:   width of the bottom line of the perspective, relative to the image width, 1.0 is full width
        twidth:   width of the top line of the perspective, relative to the image width, 1.0 is full width
        wscale:   scale of the top and the bottom width parameters
        
        """

        self.in_w, self.in_h = in_size[:2]
        self.out_w, self.out_h = out_size[:2]

        self.tl = (self.in_w // 2 * (1 + toffset - twidth * wscale), self.in_h * (1 - margin - height))  # top left
        self.bl = (self.in_w // 2 * (1 + boffset - bwidth * wscale), self.in_h * (1 - margin))           # bottom left
        self.tr = (self.tl[0] + self.in_w * twidth * wscale, self.tl[1])  # top right
        self.br = (self.bl[0] + self.in_w * bwidth * wscale, self.bl[1])  # bottom right

        self.input_pts = np.float32([self.bl, self.tl, self.tr, self.br])
        self.output_pts = np.float32(
            [[0, self.out_h-1], [0, 0], [self.out_w-1, 0], [self.out_w-1, self.out_h-1]])
        
        self.M = cv2.getPerspectiveTransform(self.input_pts, self.output_pts)
        self.M_inv = cv2.getPerspectiveTransform(self.output_pts, self.input_pts)


    def __call__(self, frame: cv2.Mat) -> cv2.Mat:
        """ Take a front view image and transform to top view """
        transformed = cv2.warpPerspective(frame, self.M, (self.out_w, self.out_h), flags=cv2.INTER_LINEAR)
        
        if cfg.DEBUG:
            p_mid_top = np.int32(((self.tl[0] + self.tr[0]) // 2, self.tl[1]))
            p_mid_bottom = np.int32(((self.bl[0] + self.br[0]) // 2, self.bl[1]))
            lpts = self.input_pts.reshape((-1, 1, 2)).astype(np.int32)
            lines = cv2.polylines(frame, [lpts],
                                  True, (255, 0, 0), 2, cv2.LINE_AA)
            lines = cv2.line(lines, p_mid_bottom, p_mid_top, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.imshow('lines', lines)

        if cfg.DEBUG:
            cv2.imshow('flat view', transformed)

        return transformed

    def inv(self, frame: cv2.Mat) -> cv2.Mat:
        """ Take a top view image and transform it to front view """
        return cv2.warpPerspective(frame, self.M_inv, (self.in_w, self.in_h), flags=cv2.INTER_LINEAR)
