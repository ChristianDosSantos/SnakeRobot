%Initializing Parameters
Iga = 0;
MaxIt= 34;
MutRate = 0;
Npop = 6;
MaxAmp = 0.1;
FitSel = 1;
ValA = 0;
ValB = 0;
% FitFun = @(a,b,c) exp( 100*(a-MaxAmp)/(0.1*MaxAmp) + 10*b + 100*(c-MaxAmp)/(0.01*MaxAmp) );
FitFun = @(x) 1 / (1 + x);
paso = 0.00001;
e = -0.1:paso:0.1;
Nant = 0;
Ncon = 10;
Cost = zeros(Npop,1);
tpaso = 0.01;
MaxCost = zeros(Iga+1);
MeanCost = zeros(Iga+1);
imax = length(e);
ConCur = zeros(1,imax);
parAmp = 200000;
conf = 0;
%tmax = 50;

%Creating First Gen 
%antPar = parAmp*(2*rand(Npop,Nant)-1);
conPar = parAmp*(2*rand(Npop,Ncon)-1);
Pop = [conPar];

%Calculating Fitness
for i = 1:Npop
    %disp(i)
    while conf == 0
        try
            ENG = gbellmf( e, [0.025 2.5 -0.1] );
            ENP = gbellmf( e,  [0.025 2.5 -0.05] );
            EC = gbellmf( e,  [0.025 2.5 0] );
            EPP = gbellmf( e,  [0.025 2.5 0.05] );
            EPG = gbellmf( e,  [0.025 2.5 0.1] );
        %     subplot(2,1,2)
        %     plot(e,ENG,e,ENP,e,EC,e,EPP,e,EPG, 'lineWidth', 3);
        %     axis( [-20 20 0 1] );
    
            for j=1:imax
                W = [ENG(j) ENP(j) EC(j) EPP(j) EPG(j)];
                f1 = Pop(i,1)*e(j) + Pop(i,2);
                f2 = Pop(i,3)*e(j) + Pop(i,4);
                f3 = Pop(i,5)*e(j) + Pop(i,6);
                f4 = Pop(i,7)*e(j) + Pop(i,8);
                f5 = Pop(i,9)*e(j) + Pop(i,10);
                ConCur(j) = min(max((W(1)*f1 + W(2)*f2 + W(3)*f3 + W(4)*f4 + W(5)*f5)  / sum(W), -24), 24);
            end
%             figure()
%             subplot(2,1,1)
%             plot(e,ConCur,'linewidth',3);
%             title('Curva de control');
%             ylabel('Voltaje');
%             xlabel('Error');
%             subplot(2,1,2)
%             plot(e,ENG,e,ENP,e,EC,e,EPP,e,EPG, 'lineWidth', 3);
%             title('Funciones de membresía');
%             ylabel('Grado de Pertenencia');
%             xlabel('Error')
%             axis( [-0.1 0.1 0 1] );
%             legend('ENG', 'ENP', 'EC', 'EPP', 'EPG');

            %sim('SnakeRobotPMatlab');
            sim('SnakeRobotPMatlab');
            %sim('SnakeRobotPMatlab_1_2_PairedWithFuzzy_2');
            disp('success');
            conf = 1;
        catch
            Pop(i,:) = parAmp*(2*rand(1, Nant + Ncon)-1);
            %disp(i)
        end 
    end
    conf = 0;
%     tssIndex = find(pos >= (1.05*MaxAmp) | pos <= (0.95*MaxAmp) );
%     tssIndex = tssIndex(end);
%     tss = tssIndex * tpaso;
%     Vss = pos(end);
      Mp = ( max(O1)+max(O2) )/2;
      ess = ( (max( O1(end-(1/tpaso):end) ) - MaxAmp) + (max( O2(end-(1/tpaso):end) ) - MaxAmp) )/2;
    if FitSel == 0
        Err = MSE;
        Err2 = MSE2;
        ErrVal = (MSE2(end)+ MSE(end)) / 2;
        Cost(i) = FitFun(ErrVal / tmax);
    else
        ErrItae1 = ITAE;
        ErrItae2 = ITAE2;
        ErrValItae = (ErrItae1(end)+ ErrItae2(end)) / 2;
%         Cost(i) = FitFun(ErrValItae);
        Cost(i) = FitFun(ErrValItae + ValA*ess + ValB*Mp);
    end
    
    %Cost(i) = FitFun(Err(end) / tmax);
end

[Cost,Ind] = sort(Cost, 'descend');
Pop = Pop(Ind,:);
MaxCost(1) = Cost(1);
MeanCost(1) = mean(Cost);
Iga = 1;

%Main Cycle
while Iga <= MaxIt
     %Selection (The better 2 are kept)
    Nmate = (Npop / 2) - 1 ;
%     probs = ( (Npop / 2) + 1 - ( 1:(Npop/2) ) ) / sum( 1:(Npop/2) );
    probs = Cost(:) / sum(Cost);
    Ma = randsample(1:(Npop),Nmate,true,probs);
    Pa = randsample(1:(Npop),Nmate,true,probs);
    for ip = 1:Nmate
        while Ma(ip) == Pa(ip)
            Pa(ip) = randsample(1:(Npop),1,true,probs);
        end
    end
    
    %Crossover
%     CrosPoint = randi((Nant + Ncon));
    Pop2 = Pop;
    Beta = rand(Nmate,Nant + Ncon);
    Pop( 3:2:Npop, : ) = (1 - Beta) .* Pop2(Ma,:)  + Beta .* Pop2(Pa,:);
    Pop( 4:2:Npop, : ) = (1 - Beta) .* Pop2(Pa,:)  + Beta .* Pop2(Ma,:);
    
    for i = 3:Npop
        while conf == 0
            try
                ENG = gbellmf( e, [0.025 2.5 -0.1] );
                ENP = gbellmf( e,  [0.025 2.5 -0.05] );
                EC = gbellmf( e,  [0.025 2.5 0] );
                EPP = gbellmf( e,  [0.025 2.5 0.05] );
                EPG = gbellmf( e,  [0.025 2.5 0.1] );
            %     subplot(2,1,2)
            %     plot(e,ENG,e,ENP,e,EC,e,EPP,e,EPG, 'lineWidth', 3);
            %     axis( [-20 20 0 1] );

                for j=1:imax
                    W = [ENG(j) ENP(j) EC(j) EPP(j) EPG(j)];
                    f1 = Pop(i,1)*e(j) + Pop(i,2);
                    f2 = Pop(i,3)*e(j) + Pop(i,4);
                    f3 = Pop(i,5)*e(j) + Pop(i,6);
                    f4 = Pop(i,7)*e(j) + Pop(i,8);
                    f5 = Pop(i,9)*e(j) + Pop(i,10);
                    ConCur(j) = min(max((W(1)*f1 + W(2)*f2 + W(3)*f3 + W(4)*f4 + W(5)*f5)  / sum(W), -24), 24);
                end
                sim('SnakeRobotPMatlab');
                %sim('SnakeRobotPMatlab_1_2_PairedWithFuzzy_2');
                disp('CrossSuccess');
                conf = 1;
            catch
                Betac = rand(1,Nant + Ncon);
                if mod(i,2) == 0
                    Pop(i,:) = (1 - Betac) .* Pop2(Pa( floor( (i-1)/2 ) ),:)  + Betac .* Pop2(Ma( floor( (i-1)/2 ) ),:);
                else
                    Pop(i,:) = (1 - Betac) .* Pop2(Ma( floor( (i-1)/2 ) ),:)  + Betac .* Pop2(Pa( floor( (i-1)/2 ) ),:);
                %disp(i)
                end
            end 
        end
        conf = 0;  
        ess = ( (max( O1(end-(1/tpaso):end) ) - MaxAmp) + (max( O2(end-(1/tpaso):end) ) - MaxAmp) )/2;
        Mp = ( max(O1)+max(O2) )/2;
        if FitSel == 0
            Err = MSE;
            Err2 = MSE2;
            ErrVal = (MSE2(end)+ MSE(end)) / 2;
            Cost(i) = FitFun(ErrVal / tmax);
        else
            ErrItae1 = ITAE;
            ErrItae2 = ITAE2;
            ErrValItae = (ErrItae1(end)+ ErrItae2(end)) / 2;
%             Cost(i) = FitFun(ErrValItae);
            Cost(i) = FitFun(ErrValItae + ValA*ess + ValB*Mp);
        end
        %Cost(i) = FitFun(Err(end) / tmax);
    end
    
    %Mutation
    Mut = randsample( [0 1], Npop*Nant*Ncon, true, [ (1 - MutRate) MutRate ] );
    NumMut = sum( Mut );
    RowMut = randi( Npop-1, NumMut, 1 ) + 1;
    ColMut = randi( Nant + Ncon, NumMut, 1 );
    Pop( RowMut, ColMut ) = 30*(2*rand(NumMut)-1);
    
%     %Fitness Calculation
%     for i = 1:Npop
%         while conf == 0
%             try
%                 ENG = sigmf( e,[Pop(i,1) Pop(i,2)] );
%                 ENP = gbellmf( e, [Pop(i,3) Pop(i,4) Pop(i,5)] );
%                 EC = gbellmf( e, [Pop(i,6) Pop(i,7) Pop(i,8)] );
%                 EPP = gbellmf( e, [Pop(i,9) Pop(i,10) Pop(i,11)] );
%                 EPG = sigmf( e, [Pop(i,12) Pop(i,13)] );
%         %         subplot(2,1,2)
%         %         plot(e,ENG,e,ENP,e,EC,e,EPP,e,EPG, 'lineWidth', 3);
%         %         axis( [-20 20 0 1] );
% 
% 
%                 for j=1:imax
%                     W = [ENG(j) ENP(j) EC(j) EPP(j) EPG(j)];
%                     f1 = Pop(i,14)*e(j) + Pop(i,15);
%                     f2 = Pop(i,16)*e(j) + Pop(i,17);
%                     f3 = Pop(i,18)*e(j) + Pop(i,19);
%                     f4 = Pop(i,20)*e(j) + Pop(i,21);
%                     f5 = Pop(i,22)*e(j) + Pop(i,23);
%                     ConCur(j) = min(max((W(1)*f1 + W(2)*f2 + W(3)*f3 + W(4)*f4 + W(5)*f5)/sum(W), -24), 24);
%                 end
% 
%                 %sim('SnakeRobotPMatlab');
%                 sim('SnakeRobotPMatlab_1_2_PairedWithFuzzy_2');
%                 disp('success')
%                 conf = 1;
%             catch
%                 Pop(i,:) = parAmp*(2*rand(1, Nant + Ncon)-1);
%                 disp('error')
%             end
%         end
%         conf = 0;
% %         Mp = max(pos);
% %         tssIndex = find(pos >= (1.05*MaxAmp) | pos <= (0.95*MaxAmp) );
% %         tssIndex = tssIndex(end);
% %         tss = tssIndex * tpaso;
% %         Vss = pos(end);
%         Err = MSE;
%         Cost(i) = FitFun(Err(end) / tmax);
%     end

    [Cost,Ind] = sort(Cost, 'descend');
    Pop = Pop(Ind,:);
    MaxCost(Iga + 1) = Cost(1);
    MeanCost(Iga + 1) = mean(Cost);
    
    if Iga > 4 && MaxCost(Iga + 1) == MaxCost(Iga - 4) && mod(Iga, 5) == 0
        Pop(2:Npop, :) = parAmp*(2*rand(Npop-1,Nant + Ncon)-1);
        disp('MicroGA')
        for i = 2:Npop
            while conf == 0
                try
                    ENG = gbellmf( e, [0.025 2.5 -0.1] );
                    ENP = gbellmf( e,  [0.025 2.5 -0.05] );
                    EC = gbellmf( e,  [0.025 2.5 0] );
                    EPP = gbellmf( e,  [0.025 2.5 0.05] );
                    EPG = gbellmf( e,  [0.025 2.5 0.1] );
                %     subplot(2,1,2)
                %     plot(e,ENG,e,ENP,e,EC,e,EPP,e,EPG, 'lineWidth', 3);
                %     axis( [-20 20 0 1] );

                    for j=1:imax
                        W = [ENG(j) ENP(j) EC(j) EPP(j) EPG(j)];
                        f1 = Pop(i,1)*e(j) + Pop(i,2);
                        f2 = Pop(i,3)*e(j) + Pop(i,4);
                        f3 = Pop(i,5)*e(j) + Pop(i,6);
                        f4 = Pop(i,7)*e(j) + Pop(i,8);
                        f5 = Pop(i,9)*e(j) + Pop(i,10);
                        ConCur(j) = min(max((W(1)*f1 + W(2)*f2 + W(3)*f3 + W(4)*f4 + W(5)*f5)  / sum(W), -24), 24);
                    end

                    sim('SnakeRobotPMatlab');
                    %sim('SnakeRobotPMatlab_1_2_PairedWithFuzzy_2');
                    disp('success')
                    conf = 1;
                    ess = ( (max( O1(end-(1/tpaso):end) ) - MaxAmp) + (max( O2(end-(1/tpaso):end) ) - MaxAmp) )/2;
                    Mp = ( max(O1)+max(O2) )/2;
                    if FitSel == 0
                        Err = MSE;
                        Err2 = MSE2;
                        ErrVal = (MSE2(end)+ MSE(end)) / 2;
                        Cost(i) = FitFun(ErrVal / tmax);
                    else
                        ErrItae1 = ITAE;
                        ErrItae2 = ITAE2;
                        ErrValItae = (ErrItae1(end)+ ErrItae2(end)) / 2;
%                         Cost(i) = FitFun(ErrValItae);
                        Cost(i) = FitFun(ErrValItae + ValA*ess + ValB*Mp);
                    end
                catch
                    Pop(i,:) = parAmp*(2*rand(1, Nant + Ncon)-1);
%                     disp('error')
                end
            end
            conf = 0;
        end
    end
    
    [Cost,Ind] = sort(Cost, 'descend');
    Pop = Pop(Ind,:);
    MaxCost(Iga + 1) = Cost(1);
    MeanCost(Iga + 1) = mean(Cost);
    %Updating Gen
    Iga = Iga + 1;
    disp(Iga)
end

ENG = gbellmf( e, [0.025 2.5 -0.1] );
ENP = gbellmf( e,  [0.025 2.5 -0.05] );
EC = gbellmf( e,  [0.025 2.5 0] );
EPP = gbellmf( e,  [0.025 2.5 0.05] );
EPG = gbellmf( e,  [0.025 2.5 0.1] );
figure()
subplot(3, 2, 1)
plot(e, ENG);
title('Función de membresía para ENG');
ylabel('Grado de Pertenencia');
xlabel('Error')
subplot(3, 2, 2)
plot(e, ENP);
title('Función de membresía para ENP');
ylabel('Grado de Pertenencia');
xlabel('Error')
subplot(3, 2, 3)
plot(e, EC);
title('Función de membresía para EC');
ylabel('Grado de Pertenencia');
xlabel('Error')
subplot(3, 2, 4)
plot(e, EPP);
title('Función de membresía para EPP');
ylabel('Grado de Pertenencia');
xlabel('Error')
subplot(3, 2, 5)
plot(e, EPG);
title('Función de membresía para EPG');
ylabel('Grado de Pertenencia');
xlabel('Error')
figure()
subplot(2,1,2)
plot(e,ENG,e,ENP,e,EC,e,EPP,e,EPG, 'lineWidth', 3);
title('Funciones de membresía');
ylabel('Grado de Pertenencia');
xlabel('Error')
axis( [-0.1 0.1 0 1] );
legend('ENG', 'ENP', 'EC', 'EPP', 'EPG');

imax = length(e);
    for i=1:imax
        W = [ENG(i) ENP(i) EC(i) EPP(i) EPG(i)];
        f1 = Pop(1,1)*e(i) + Pop(1,2);
        f2 = Pop(1,3)*e(i) + Pop(1,4);
        f3 = Pop(1,5)*e(i) + Pop(1,6);
        f4 = Pop(1,7)*e(i) + Pop(1,8);
        f5 = Pop(1,9)*e(i) + Pop(1,10);
        ConCur(i) = min(max((W(1)*f1 + W(2)*f2 + W(3)*f3 + W(4)*f4 + W(5)*f5)/sum(W), -24), 24);
    end

subplot(2,1,1)
plot(e,ConCur,'linewidth',3);
title('Curva de control');
ylabel('Voltaje');
xlabel('Error');
%axis( [-21 21 -12 12] );
%sim('PrinterMotorP');
fprintf ( '%i ', Pop( 1, : ) );
figure()
plot( 0:MaxIt, MaxCost, 'b-' );
hold on
plot( 0:MaxIt, MeanCost, 'r-' );
title('Fitness de la población');
ylabel('Valor');
xlabel('Generación');
legend( 'Fitness máximo', 'Fitness Medio' );