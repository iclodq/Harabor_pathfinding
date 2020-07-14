import sys, getopt
from util import gm_parser

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

    parser = gm_parser.gm_parser()
    parser.load(inputfile)
    parser.write(inputfile)

if __name__ == "__main__":
    main(sys.argv[1:])



