import cv2

from bot_mapping import maze_converter

maze_converter_ = maze_converter()

def main():
    tiny = cv2.imread("/home/haiderabbasi/Desktop/party2.png",cv2.IMREAD_GRAYSCALE)
    print(tiny)
    cv2.namedWindow("tiny_maze",cv2.WINDOW_FREERATIO)
    cv2.imshow("tiny_maze",tiny)
    cv2.waitKey(0) 
    
    maze_converter_.one_pass(tiny)



if __name__ == '__main__':
    main()