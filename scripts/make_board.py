import cv2
import getopt
import sys

def main():
    verbose = False
    N = 1
    M = 3
    _usage = "{-o, --output} [output image file] " + \
        "{-N, --rows} [number of rows for the grid] " + \
        "{-M, --columns} [number of columns for the grid] "
    options, remainder = getopt.getopt(sys.argv[1:], 'o:N:M:v',
        ['output=', 'rows=', 'columns=', 'verbose'])
    for opt, arg in options:
        if opt in ('-o', '--output'): output_filename = arg
        elif opt in ('-N', '--rows'): N = arg
        elif opt in ('-v', '--verbose'): verbose = True
        else:
            print("Invalid argument " + opt)
            sys.exit(_usage)

    # Follow https://gregorkovalcik.github.io/opencv_contrib/tutorial_aruco_board_detection.html
    # TODO: translate to python:
    # See https://docs.opencv.org/4.5.3/dc/df7/dictionary_8hpp.html
    # dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50) # or just 0, for the simplest one
    # cv::aruco::GridBoard board = cv::aruco::GridBoard::create(M, N, 0.12, 0.02, dict);
    # board.draw(outSize, cvmat)
    # cv::imwrite(output_filename, cvmat); # save img to the outfile
    # if verbose: # show the generated grid board
    #   cv::imshow(output_filename, cvmat);
    #   cv::waitKey()

if __name__ == "__main__":
    main()