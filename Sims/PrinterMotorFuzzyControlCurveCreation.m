
paso = 0.01;
e = -20:paso:20;
%par = 100 * (2 * rand(3*3 + 2*2) - 1 ) 
a = 10;
ENG = sigmf( e,[-0.5*a -10] );
ENP = gbellmf( e, [2.5 a -7.5] );
EC = gbellmf( e, [5 a 0] );
EPP = gbellmf( e, [2.5 a 7.5] );
EPG = sigmf( e, [0.5*a 10] );
subplot(2,1,2)
plot(e,ENG,e,ENP,e,EC,e,EPP,e,EPG, 'lineWidth', 5);
axis( [-20 20 0 1] );

imax = length(e);
for i=1:imax
    W = [ENG(i) ENP(i) EC(i) EPP(i) EPG(i)];
    f1 = -12;
    f2 = 2*e(i) + 8;
    f3 = 0.4*e(i);
    f4 = 2*e(i) - 8;
    f5 = 12;
    ConCur(i) = (W(1)*f1 + W(2)*f2 + W(3)*f3 + W(4)*f4 + W(5)*f5)  / sum(W);
end

subplot(2,1,1)
plot(e,ConCur,'linewidth',3);
%axis( [-21 21 -12 12] );
%sim('PrinterMotorP');