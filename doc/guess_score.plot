set hidden3d
set title  'Guess Score'
set xlabel 'Translational error'
set ylabel 'Rotational error'
# set zlabel 'Response'
set isosamples 30
set xrange [-5:5]
set yrange [-25:25]
set zrange [0.0:1.5]
set xtics 1 
set ytics 5
set ztics 0.5
# set view 40,50,1.0,1.5
min(A,B) = A < B ? A : B
# max translation error
mt = 3
# max rotation error
ma = 20
r(x,y)= 1 - min(1, (x * x) / (mt * mt) + (y * y) / (ma * ma) )
set contour base
set surface
show contour
splot r(x,y)
set output "guess_score.eps"
set terminal postscript eps
replot
