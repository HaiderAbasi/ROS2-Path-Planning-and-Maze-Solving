import cv2

from bot_mapping import bot_mapper

bot_mapper_ = bot_mapper()

def main():
    tiny = cv2.imread("/home/haiderabbasi/Desktop/party.png",cv2.IMREAD_GRAYSCALE)
    print(tiny)
    cv2.namedWindow("tiny_maze",cv2.WINDOW_FREERATIO)
    cv2.imshow("tiny_maze",tiny)
    cv2.waitKey(0) 
    
    bot_mapper_.one_pass(tiny)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()