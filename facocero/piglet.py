import sys, getopt
from domains import gridmap

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

    parser = gridmap.gridmap()
    parser.load(inputfile)
    parser.write()

if __name__ == "__main__":
    main(sys.argv[1:])



