# Olivier Stasse
# JRL/CNRS Copyright
# Gnuplot file to display analysis of 
# walkGenJRL computation.
# 
plot "jl_pgleftfootref.dat" u ($0-33720):5 w l t "LF-Ref-X", \
  "jl_pgrightfootref.dat"  u ($0-33720):5 w l t "RF-Ref-X", \
  "RebuildWaist.dat" u 1 w l t "Waist-R-X", \
  "RebuildWaist.dat" u 7 w l t "Waist-A-X", \
  "RebuildZMP.dat" u ($11*0.06) w l t "ChangeSupport"		
