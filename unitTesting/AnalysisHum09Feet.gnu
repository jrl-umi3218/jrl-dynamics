# Olivier Stasse
# JRL/CNRS Copyright
# Gnuplot file to display analysis of
# walkGenJRL computation.
#
set multiplot layout 2,1 columnsfirst

plot \
  "RebuildWaist.dat" u 1 w l t "Waist-R-X", \
  "RebuildWaist.dat" u 7 w l t "Waist-A-X", \
  "RebuildZMP.dat" u 5 w l t "LF-R-X", \
  "RebuildZMP.dat" u 8 w l t "RF-R-X", \
  "RebuildZMP.dat" u 11 w l t "SFP-A-X"

plot "jl_pgleftfootref.dat" u ($0-33720):13 w l t "LF-Ref-X", \
  "jl_pgrightfootref.dat"  u ($0-33720):13 w l t "RF-Ref-X", \
  "RebuildZMP.dat" u 7 w l t "LF-A-Z", \
  "RebuildZMP.dat" u 10 w l t "RF-A-Z", \
  "RebuildZMP.dat" u ($14*0.06+0.1) w l t "ChangeSupport"
unset multiplot