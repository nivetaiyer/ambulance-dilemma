import os

run_num = 0
for depth in range(1,10):
    for iterations in range(1,100,20):
        run_num += 1

        julia_runner = '''
        include("testing.jl")
        the_test(%d,%d)
        ''' % (depth, iterations)

        qsubscript = '''
        #!/bin/bash

        #$ -N ambulance_test_depth%d_iter%d
        #$ -o job_depth%d_iter%d.out
        #$ -e job_depth%d_iter%d.error
        #$ -cwd
        #$ -S /bin/bash

        julia runner%d.jl ''' % (depth, iterations, depth, iterations, depth, iterations, run_num)

        runfile = open('runner%d.jl' % run_num, 'w' )
        runfile.write(julia_runner)
        runfile.close()

        qsubfile = open('run%d.submit' % run_num, 'w')
        qsubfile.write(qsubscript)
        qsubfile.close()

        os.system('qsub run%d.submit' % run_num)
