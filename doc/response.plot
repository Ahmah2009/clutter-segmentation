# set hidden3d
set title 'Response'
set xlabel 'Translational error'
set ylabel 'Rotational error'
# set zlabel 'Response'
set isosamples 30
set xrange [-1.5:1.5]
set yrange [-1.5:1.5]
set zrange [0.0:1.5]
set xtics 0.5
set ytics 0.5
set ztics 0.5
set view 40,50,1.0,1.5
min(A,B) = A < B ? A : B
r(x,y)= min(1, x * x + y * y)
set contour base
set surface
show contour
splot r(x,y)
set output "datei.eps"
set terminal postscript eps
replot
