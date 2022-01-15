import cv2 as cv
import getopt
import sys

def main():
    verbose = False
    N = 2
    M = 3
    len = 0.15 # [m]
    gap = 0.01 # [m]
    margin = 8 # [pixel]
    offset = 0
    _usage = "{-o, --output} output image file\n" + \
        "{-N, --rows} number of rows for the grid\n" + \
        "{-M, --columns} number of columns for the grid\n" + \
        "--length length of 1 ARUCO marker [m]]\n" + \
        "--gap gap between markers [m]\n" + \
        "--margin minimum margin in the output image [pixel]\n" + \
        "--offset The marker number to start the pattern at\n" + \
        "\nexample argument:\n" + \
        "--length 0.1 --gap 0.005 -N 12 -M 16 --offset 28 -o worlds/gazebo_world_description/materials/textures/bottom_marker.jpg"
    options, remainder = getopt.getopt(sys.argv[1:], 'o:N:M:v',
        ['output=', 'rows=', 'columns=', 'length=', 'gap=', 'offset=', 'verbose'])
    for opt, arg in options:
        if opt in ('-o', '--output'): output_filename = arg
        elif opt in ('-N', '--rows'): N = int(arg)
        elif opt in ('-M', '--columns'): M = int(arg)
        elif opt=='--offset': offset = int(arg)
        elif opt=='--margin': margin = int(arg)
        elif opt=='--length': len = float(arg)
        elif opt=='--gap': gap = float(arg)
        elif opt in ('-v', '--verbose'): verbose = True
        else: sys.exit(_usage)

    # Follow https://gregorkovalcik.github.io/opencv_contrib/tutorial_aruco_board_detection.html
    # See https://docs.opencv.org/4.5.3/dc/df7/dictionary_8hpp.html
    dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    board = cv.aruco.GridBoard_create(M, N, len, gap, dict, offset)
    dim = (int(1024 * (M*len + (M+1)*gap)), int(1024 * (N*len + (N+1)*gap)))
    img = board.draw(dim, marginSize=margin, borderBits=1)
    cv.imwrite(output_filename, img); # save img to the outfile
    if verbose: # show the generated grid board
      cv.imshow(output_filename, img)
      cv.waitKey(0)

if __name__ == "__main__":
    main()