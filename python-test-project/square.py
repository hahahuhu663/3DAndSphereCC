board = ["b", "w"]
input = "RRLL"
currentTurn = 1

def main(lines):
    currentTurn = 1
    board = ["b","w"]
    bCount = 1
    wCount = 1
    def makeMove(turn):
        if turn%2 == 0:
            return "w"
        else:
            return "b"

    def count(target):
        if target == "b":
            bCount +=1
        else:
            wCount+=1
    
    def flip(f,t, target):
        for i in range(f, t):
            board[i] = target
            count(target)
            
    for move in lines[0]:
        new = makeMove(currentTurn)
        if move == "R":
            if  board[-1] != new:
                for i in range(len(board)-2,-1,-1):
                    if board[i] == new:
                        flip(i+1,len(board),new)
                        break
            board.append(new)
        else:
            if  board[0] != new:
                for i in range(1,len(board)):
                    if board[i] == new:
                        flip(0,i,new)
                        break
            board.insert(0,new)
        count(new)
        currentTurn+=1
    print(str(board.count("b"))+" "+ str(board.count("w")))

lines = "RRLL"
main(lines)