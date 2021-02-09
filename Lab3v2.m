clear all;
close all;
clc;

%% Common Parameters
X_coord=[1.5 3.5 5.5 7.5 4.5 1.5 3.5 5.5 7.5 4.5 1.5 3.5 5.5 7.5 0 0 0 0 0 0 0 0 0 0 0];
Y_coord=[0 0 0 0 0 0 0 0 0 0 0 0 0 0 1.5 3.5 5.5 2.5 1.5 3.5 5.5 2.5 1.5 3.5 5.5];
Z_coord=[0.5 0.5 0.5 0.5 1.5 2.5 2.5 2.5 2.5 3.5 4.5 4.5 4.5 4.5 0.5 0.5 0.5 1.5 2.5 2.5 2.5 3.5 4.5 4.5 4.5];

P_M = [X_coord; Y_coord; Z_coord; ones(1,25)];

f =636;
cx = 317;
cy =240;
K = [ f 0 cx; 0 f cy; 0 0 1 ]; % intrinsic parameter matrix

I = zeros(cx*2,cy*2);
% figure(1), imshow(I,[]), title('View 1');
% figure(2), imshow(I,[]), title('View 1');

%% Image1 Parameters

R1 = [0.836360786907943,-0.548009355736070,0.0136521115020191;
        0.0890938602340798,0.111315577807920,-0.989783373373123;
        0.540890856080334,0.829032320336804,0.141924253197462];
    
Mext1 = [0.836360786907943,-0.548009355736070,0.0136521115020191,8.45333392971705;
        0.0890938602340798,0.111315577807920,-0.989783373373123,17.4056067645161;
        0.540890856080334,0.829032320336804,0.141924253197462,59.0472004066266];

Pmorg_c1 = Mext1(:,4);

H_m_c1 = [R1 Pmorg_c1 ; 0 0 0 1];
    
%% Image2 Parameters

R2 = [0.351565848936264,-0.936153740605348,-0.00419854883731698;
        0.124250854434069,0.0511057524815728,-0.990933866227054;
        0.927881015558637,0.347856832639389,0.134284939408575];
    
Mext2 = [0.351565848936264,-0.936153740605348,-0.00419854883731698,-16.3279821273850;
         0.124250854434069,0.0511057524815728,-0.990933866227054,16.8507372562537;
         0.927881015558637,0.347856832639389,0.134284939408575,51.8341461747324];

Pmorg_c2 = Mext2(:,4);
    
H_m_c2 = [R2 Pmorg_c2 ; 0 0 0 1];

%% Part A - Calculate the essential matrix

%Calculate H_c2_c1 from the known H_m_c1 and H_m_c2

H_c2_c1 = H_m_c1 / H_m_c2; 

% From that extract R_c2_c1 and Pc2org_c1
R_c2_c1 = [0.806999579268468,0.0623837722637700,0.587247770552310;
            -0.0687304821128553,0.997568719529593,-0.0115226144873010;
            -0.586538830674534,-0.0310630773464351,0.809325203695473];

Pc2org_c1 = [-9.86069069824507;
              0.0708731767700921;
              8.04295970584477];

% Calculate E from Pc2org_c1 and R_c2_c1 using the skew symmetric matrix technique.

t = Pc2org_c1;
E = [0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0] *R_c2_c1;
disp('true E = ');
disp(E);

%% Part B - Draw epipolar lines

p1=Mext1*P_M;
p1(1,:)=p1(1,:)./p1(3,:);
p1(2,:)=p1(2,:)./p1(3,:);
p1(3,:)=p1(3,:)./p1(3,:);

u1=K*p1;

figure(1), imshow(I,[]), title('View 1');
%convert image points from normalized to unnormalized
for i=1:length(u1)
    rectangle('Position',[u1(1,i)-2 u1(2,i)-2 4 4], 'FaceColor', 'r');
end
% pause

%Compute el=E*p2 where el=[a,b,c] and the equation of the line is ax+by+c=0
p2=Mext2*P_M;
p2(1,:)=p2(1,:)./p2(3,:);
p2(2,:)=p2(2,:)./p2(3,:);
p2(3,:)=p2(3,:)./p2(3,:);

figure(2); imshow(I,[]); title('View 2');
%convert image points from normalized to unnormalized
u2=K*p2;
for i=1:length(u2)
    rectangle('Position',[u2(1,i)-2 u2(2,i)-2 4 4], 'FaceColor', 'y');
end
% pause

%Draw epipolar lines
for i=1:length(p2)
    
    figure(2);
    rectangle('Position',[u2(1,i)-2 u2(2,i)-2 4 4], 'FaceColor', 'r');
    
    %Compute el=E*p2 where el=[a,b,c] and the equation of the line is ax=by=c=0
    figure(1);
    el=E*p2(:,i);
    
    %To find two points we assume x =1 and solve for y then assume x=2 and solve for y
    px=1;
    pLine0=[px; (-el(3)-el(1)*px)/el(2); 1];
    px=-2;
    pLine1=[px; (-el(3)-el(1)*px)/el(2); 1];
    
    %Convert to unnormalizedp
    Line0=K*pLine0; 
    pLine1=K*pLine1;
    
    %Draw the line
    line([pLine0(1) pLine1(1)], [pLine0(2) pLine1(2)], 'Color', 'b');
%     pause;
    
end

%% Part C - Use 8 point algorithm to compute E

%Calculate the true essential matrix

% Apply preconditioning
xn=p1(1:2, :); %xn is a 2xN matrix  
N=size(xn,2);
t=(1/N)* sum(xn,2); %(x,y) centroid of the points
xnc=xn-t*ones(1,N); % center the points
%dc is 1xN vector = distance of each new position to (0,0)
dc=sqrt(sum(xnc.^2)); 
davg=(1/N)*sum(dc); %average distance to the origin
s=sqrt(2)/davg; %the scale factor so that the avg dist is sqrt(2)          
T1=[s*eye(2), -s*t; 0 0 1]; %transformation matrix 
p1s=T1*p1;

xn=p2(1:2, :);
N=size(xn,2);
t=(1/N)* sum(xn,2); %(x,y) centroid of the points
xnc=xn-t*ones(1,N);
dc=sqrt(sum(xnc.^2));
davg=(1/N)*sum(dc);
s=sqrt(2)/davg;
T2=[s*eye(2), -s*t; 0 0 1];
p2s=T2*p2;

A = [p1s(1,:)'.*p2s(1,:)' p1s(1,:)'.*p2s(2,:)' p1s(1,:)' ...
    p1s(2,:)'.*p2s(1,:)' p1s(2,:)'.*p2s(2,:)' p1s(2,:)' ...
    p2s(1,:)' p2s(2,:)' ones(length(p1s),1)];

[U,D,V]=svd(A);
x=V(:,size(V,2));
Escale=reshape(x,3,3)';
    
%postcondition
[U,D,V] = svd(Escale);
%D should be[1 0 0], we enforce that by replacing 
%it as shown in this line
Escale=U*diag([1 1 0])*V'; 
%undo scaling done in preconditioning using
E=T1' * Escale * T2;

disp('Calculated E = ');
disp(E);
%Although the above is different but as we said it is up to a scale, hence 
%divide by E(1,2)
disp('calculated E after scaling= ');
disp(E/(-E(1,2)));
