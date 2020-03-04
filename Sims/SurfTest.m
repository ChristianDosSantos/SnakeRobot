for i = -0.2:0.01:0.2
    for j = -0.1:0.01:0.1
%         disp((i+0.2)/0.01 + 1)
%         disp((j+0.1)/0.01 + 1)
        z2(round((i+0.2)/0.01 + 1),round((j+0.1)/0.01 + 1)) = sin(i) + j;
    end
end
[x,y] =  meshgrid(-0.2:0.01:0.2,-0.1:0.01:0.1);
f = @(x,y) sin(x) + y;
z = f(x,y);
figure()
surf(x,y,z);
figure()
surf(x,y,z2');