#!/home/amit/Software/miniconda3/envs/default/bin/python

import scipy.spatial as sp
import csv
import numpy as np
import argparse
import multiprocessing as mp
from pointcloudvolume import calculateVolume

"""
# The function that calculates volume
def writevolume(input):
    infile, outdir = input
    outfilename = outdir + '/volume.dat'
    with open(infile,'r',newline='') as inp, open(outfilename,'w') as outfile:
        outfile.write('volume\n')
        inp.readline()
        reader = csv.reader( inp, delimiter=',', quoting=csv.QUOTE_NONNUMERIC )
        for row in reader:
            # Read the position and rotation vectors
            xr = np.array(row).reshape((-1,3))
            # Get only the position vectors
            xi = xr[:int(xr.shape[0]/2),:]
            # Project the position vectors to a sphere
            x = xi/np.linalg.norm(xi, axis=1)[:,np.newaxis]
            # Make the mesh
            mesh = sp.ConvexHull(x).simplices
            # Calculate volume of each tetrahedron
            a = xi[mesh[:,0]]
            b = xi[mesh[:,1]]
            c = xi[mesh[:,2]]
            vol = 0.1666667*np.sum( np.abs( np.einsum('ij,ij->i',np.cross(a,b,axis=1),c) ) )
            line = '{0:7.5f}\n'.format(vol)
            outfile.write(line)
"""
# The function that calculates volume
def writevolume(input):
    infile, outdir = input
    outfilename = outdir + '/volume.dat'
    with open(infile,'r',newline='') as inp, open(outfilename,'w') as outfile:
        outfile.write('volume\n')
        inp.readline()
        reader = csv.reader( inp, delimiter=',', quoting=csv.QUOTE_NONNUMERIC )
        for row in reader:
            vol = calculateVolume( row )
            line = '{0:7.5f}\n'.format(vol)
            outfile.write(line)

if __name__ == "__main__":

    # Create a command-line argument parser
    parser = argparse.ArgumentParser(description='Input data file path.')
    parser.add_argument('-i','-input', nargs='+', required=True,
                         help='a single or a list of input data files')
    parser.add_argument('-o','-output', nargs='+', required=True,
                        help='output directories corresponding to the input files')

    # Parse the input files
    args = parser.parse_args()

    if( len(args.i) != len(args.o) ):
        print('The number of output directories do not match input files.')
        exit()

    p = mp.Pool(processes=8)
    p.map( writevolume, zip(args.i, args.o) )
