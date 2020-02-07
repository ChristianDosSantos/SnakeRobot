figure()
plot(t,O1);
title('Posicion lineal de la articulacion 1');
xlabel('Tiempo(s)');
ylabel('Distancia(m)');
hold on 
plot(t,O1Ref);
legend('O1','O1Ref');
figure()
plot(t,O2);
title('Posicion lineal de la articulacion 2');
xlabel('Tiempo(s)');
ylabel('Distancia(m)');
hold on 
plot(t,O2Ref);
legend('O2','O2Ref');
figure()
plot(t,Vn);
ylabel('Velocidad en el eje normal (m/s)');
xlabel('Tiempo(s)');
title('Velocidad del centro de masa en el eje normal al desplazamiento');
figure()
plot(t,Vt);
ylabel('Velocidad en el eje transversal (m/s)');
xlabel('Tiempo(s)');
title('Velocidad del centro de masa en el eje transversal al desplazamiento');
figure()
plot(t,Theta);
title('Orientacion absoluta');
ylabel('Theta (rad)');
xlabel('Tiempo (s)');
figure()
plot(t,Pt);
title('Posicion en el eje transversal del centro de masa');
ylabel('Distancia (m)');
xlabel('Tiempo (s)');
figure()
plot(t,Pn);
title('Posicion en el eje Normal del centro de masa');
ylabel('Distancia (m)');
xlabel('Tiempo (s)');
figure()
plot(Px,Py);
title('Trayectoria del centro de masa del robot serpiente');
ylabel('Y (m)');
xlabel('X (m)');