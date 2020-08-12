clc;close all;clear all;
disp("==== Started ====");
tic
%% ==== Black Magic Var's ==== %%
ws=get(0,'screensize');
w1=ws(3)/2;w2=ws(4)/2;w3=w2+20;w4=w2-120;

%% ==== Program Var's ====%%
print_img=true;
c_path=cd; % Get curr dir
folder_name='results'; % Destination IMGs Folder Name
if ~exist(folder_name, 'dir')mkdir(folder_name);end % Creates Folder if not found
result_folder=strcat(cd,'\',folder_name,'\'); % Joins paths in String

%% ==== SRA plotter ==== %%
% ===== Input Vars ====   %
with_robot =0 ; % set to true to use Serial
ex = 2; % Runs Selected Exercise Code in C++
sala = 'Sala12.png';
xf = 5.0*5.0 ;yf = (80-75)*5.0 ; rho_min=0.2; % Ex1 Final Point
sa_angle=5;
v_thr=.15;

I=imread(strcat(c_path,'/mapping/',sala));
I = flip(I ,1); % vertical flip
        
        
vmin=15;
if(with_robot==true)
    global s;
    %delete(instrfindall);
    %s = Bluetooth('H-C-2010-06-01',1);
    %fopen(s);
    s=start_serial();
end

[x, y, phi, u, v,rho,t,ve,vd,vmm,Fai,Faj,Fri,Frj,Fi,Fj] = deal(0);

if(with_robot==false)
    switch ex
        case 1
            disp('==== Exercise 1 - Move To Point ====');
            %system('ex1.exe'); % Corre o Codigo .EXE
            M = csvread(strcat(result_folder,'VFF.csv')); % FFV
        case 2
            disp('==== Exercise 2 - Move With Line ====');
            %system('ex2.exe'); % Corre o Codigo .EXE
            M = csvread(strcat(result_folder,'VFH.csv'));
            T1 = readtable(strcat(result_folder,'Hist.csv'));
            T2 = readtable(strcat(result_folder,'Vales.csv'));
            H=table2array(T1(:,1))';
            Hs=zeros(1,72);
            Hs=[Hs,table2array(T1(:,2))'];
            V=zeros(1,72);
            V=[V,table2array(T2(:,1))'];
        case 3
            disp('==== Exercise 3 - Move To Point With Pose ====');
            %system('ex3.exe'); % Corre o Codigo .EXE
            M = csvread(strcat(result_folder,'ex3.csv'));
    end
else
    disp('==== Running in Serial Mode ====');
end
% Show Figures
figure('Name','Trajetória Robot','NumberTitle','off','Position',[w1-3/2*w3,w2,w3,w4]);% [2x3][1]'Position',[w1-3/2*w3,w2-w4/2,w3,w4]);% [1x3][1] %'Position',[w1-w3,w2-w4/2,w3,w4]);% [1x2][1]
h=imshow(imresize(I,5));
uistack(h, 'bottom');
figure('Name','Distância ao Objectivo','NumberTitle','off','Position',[w1-1/2*w3,w2,w3,w4]);% [2x3][2] 'Position',[w1-1/2*w3,w2-w4/2,w3,w4]);% [1x3][2] %'Position',[w1,w2-w4/2,w3,w4]);% [1x2][2]
figure('Name','Velocidade','NumberTitle','off','Position',[w1+1/2*w3,w2,w3,w4]);% [2x3][3] 'Position',[w1+1/2*w3,w2-w4/2,w3,w4]);% [1x3][3]
if ex==2
    figure('Name','Histogram','NumberTitle','off','Position',[w1-1/2*w3,w2-(w4+80),w3,w4]);% [2x3][5]
    figure('Name','Valleys','NumberTitle','off','Position',[w1+1/2*w3,w2-(w4+80),w3,w4]);% [2x3][6]
    figure('Name','Compass','NumberTitle','off','Position',[w1-3/2*w3,w2-(w4+80),w3,w4]);% [2x3][4]
end

% Start Read code
i=1;index=1;
try 
    while (true)
        if(with_robot==true)
            data=fscanf(s);
        else
            data=strcat(num2str(M(i,1)),',',num2str(M(i,2)),',',num2str(M(i,3)) ...
                ,',',num2str(M(i,4)),',',num2str(M(i,5)),',',num2str(M(i,6)),',',num2str(M(i,7)) ...
                ,',',num2str(M(i,8)),',',num2str(M(i,9)),',',num2str(M(i,10)),',',num2str(M(i,11))...
                ,',',num2str(M(i,12)),',',num2str(M(i,13)));
        end
        %disp(data);
        D = strsplit(data,',');
        
        t=[t,str2num(D{1})];
        x=[x,str2num(D{2})];
        y=[y,str2num(D{3})];
        phi=[phi,str2num(D{4})];
        rho=[rho,str2num(D{5})];
        ve=[ve,str2num(D{6})];
        vd=[vd,str2num(D{7})];
        vmm=[vmm,vmin];
        Fai=[Fai,str2num(D{8})];
        Faj=[Faj,str2num(D{9})];
        Fri=[Fri,str2num(D{10})];
        Frj=[Faj,str2num(D{11})];
        Fi=[Fi,str2num(D{12})];
        Fj=[Fj,str2num(D{13})];

        figure(1);
        plot(x(2:i),y(2:i),'LineWidth',1.5)
        title('Trajectory in Real Time');xlabel('X Pos');ylabel('Y Pos');
        hold on;
        if(i==1) % Plot Starting Point
            x0=x(i+1);
            y0=y(i+1);
        end 
        plot(x0, y0, 'ro','LineWidth',2);
        plot(xf, yf, 'bo','LineWidth',2);

        % Plot Vectores
        [xv,xy]=pol2cart(phi(i+1),1);
        u=[u,xv];
        v=[v,xy];
        quiver(x(2:i+1),y(2:i+1),u(2:i+1),v(2:i+1),.3); % Vector phi plot
        quiver(x(2:i+1),y(2:i+1),Fai(2:i+1),Faj(2:i+1),'color',[0 1 0]); % Vector Fa
        quiver(x(2:i+1),y(2:i+1),Fri(2:i+1),Frj(2:i+1),'color',[1 0 0]); % Vector Fr
        quiver(x(2:i+1),y(2:i+1),Fi(2:i+1),Fj(2:i+1),'color',[0 0 1]); % Vector F
        uistack(imshow(imresize(I,[400 400],'lanczos3')), 'bottom');
        hold off;
        
        % Plot Distancia ao Objective
        figure(2);
        plot(t(2:i+1),rho(2:i+1),'LineWidth',1.5,'Color','red');
        title('Distance to Object');xlabel('Time in (us)');ylabel('Distance \rho');
        hold off;

        % Plot Velocidade Rodas
        figure(3);
        plot(t(2:i+1),ve(2:i+1),'LineWidth',1.5,'Color','blue');
        hold on;
        plot(t(2:i+1),vd(2:i+1),'LineWidth',1.5,'Color','green');
        hold on;
        plot(t(2:i+1),vmm(2:i+1),'--','LineWidth',1.5,'Color','k');
        plot(t(2:i+1),-vmm(2:i+1),'--','LineWidth',1.5,'Color','k');
        hold off;
        title('Wheel Velocitys');xlabel('Time in (us)');ylabel('Values');
        if ex==2 
        % Plot Historama
            figure(4);
            %polarplot(Hs,'Color','red','Marker','square')
            %polarplot(Hs((360/sa_angle)*(i-1):(i-1)*(360/sa_angle)),'Color','red','Marker','square');
            if(i==1),polarplot(Hs(1:360/sa_angle),'Color','red','Marker','square');
            else
                polarplot(Hs((360/sa_angle)*(i):(360/sa_angle)*(i)+(360/sa_angle)),'Color','red','Marker','square');
                % ***** Histograma Normal Em Grafico de Barras **** %
                % bar([0:sa_angle:360-sa_angle],H((360/sa_angle)*(i):(360/sa_angle)*(i)+(360/sa_angle)-1));xlim([0, 360]);
                
                % ***** Histograma Suavisado Em Grafico de Barras **** %
                % bar([0:sa_angle:360-sa_angle],Hs((360/sa_angle)*(i):(360/sa_angle)*(i)+(360/sa_angle)-1),'FaceColor','red','FaceAlpha',.3);xlim([0, 360]);
            end
            % + Threshold
            hold on;th = linspace(0,2*pi,50);polarplot(th,v_thr+zeros(size(th)),'--','Color','black');hold off;
            %disp(Hs(1+(i-1):(i-1)+(360/sa_angle)));
            ax=gca;ax.ThetaZeroLocation = 'top';%ax.ThetaDir = 'clockwise';
            %polarhistogram(Hs(1+((i-1)*sa_angle):((i-1)*sa_angle)+(360/sa_angle)),360/sa_angle)
            % Plot Valleys
            figure(5);
            if(i==1)
                polarplot(V(1:360/sa_angle),'Color','green','Marker','square');
            else
                polarplot(V((360/sa_angle)*(i):(360/sa_angle)*(i)+(360/sa_angle)),'Color','green','Marker','square');
            end
            ax2=gca;ax2.ThetaZeroLocation = 'top';ax2.ThetaDir = 'clockwise';
            %polarplot(V,'Color','green','Marker','square')
            %polarhistogram(V(1+((i-1)*sa_angle):((i-1)*sa_angle)+(360/sa_angle)),360/sa_angle)
        end
        
        figure(6);
        compass(u(i+1),v(i+1));
        camorbit(0,180)
        %camroll(90)
        i=i+1;
    end
    %Catch Error
    catch ME
        figure(3);
        legend({'V roda esq','v roda dir'},'Location','southwest')
        figure(1);
        if(ex==1),legend({'Traj','Start','End','Phi','Fa','Fr','F'},'Location','southwest');end
        hold on
        uistack(imshow(imresize(I,5)), 'bottom');
       
        fprintf("\n Elapsed(seg): ");
        disp(toc);
        disp("==== Finished ====");
        %== Spit Error ==%
        rethrow(ME)
end
    
function s=start_serial()
    delete(instrfindall);
    s = serial('COM16','BaudRate',9600);
    %s = Bluetooth('H-C-2010-06-01',1);
    fopen(s);
end

function y=close_serial(s)
    fclose(s)
    delete(s)
    clear s
    delete(instrfindall);
end

function D = strparser(D)
    D = strsplit(data,', ')
end

% T = readtable('results/Hist.csv');
% theta = T(1:360/sa_angle,1)
% polarhistogram(T(1:360/sa_angle,1),72)