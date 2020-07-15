import sys, getopt
from domains import gridmap, grid_action

def main(argv):
    inputfile = ''
    outputfile = ''
    try:
      opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
      print('test.py -i <inputfile>')
      sys.exit(2)
    for opt, arg in opts:
      if opt == '-h':
         print('test.py -i <inputfile>')
         sys.exit()
      elif opt in ("-i", "--ifile"):
         inputfile = arg
    print('Input file is ', inputfile)

    ga = grid_action.grid_action();
    gm = gridmap.gridmap()
    gm.load(inputfile)
    gm.write()

    print("1,1")
    for move in gm.get_moves(1,1):
        move.print()

    print("2,2")
    for move in gm.get_moves(2,2):
        move.print()

    print("0,0")
    for move in gm.get_moves(0,0):
        move.print()
        
if __name__ == "__main__":
    main(sys.argv[1:])



