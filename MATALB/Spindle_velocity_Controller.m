clear all;
close all;

%make stage velocity controller
J =  0.0031;
D =  0.0267;
ts=5*10^-4; %sampling period

taum=J/D;

%PZ cancel PI
omega0=2*pi*80; %bandwithd[rad/s]
tau=taum;

KP=omega0*J;
KI=KP/taum;

%ss
Gc=KP+tf(1,[1 0])*KI;
ss_ctrl_d=c2d(Gc,ts);

[Afbi, Bfbi, Cfbi, Dfbi] = ssdata(ss_ctrl_d);
%%1
FILENAME = 'spindle_velocity_controler.txt';
FID = fopen(FILENAME,'w');

fprintf(FID,'float Aspvpi[1]= {%.20e};\n',Afbi);
fprintf(FID,'float Bspvpi[1]= {%.20e};\n',Bfbi);
fprintf(FID,'float Cspvpi[1]= {%.20e};\n',Cfbi);
fprintf(FID,'float Dspvpi[1]= {%.20e};\n',Dfbi);
fclose(FID);


