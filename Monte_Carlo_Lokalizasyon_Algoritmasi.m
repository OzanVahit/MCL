% produced by Ozan Vahit Altınpınar (ozan.altinpinar@cumhuriyet.edu.tr, altinpinaro@itu.edu.tr) (2025)
% Matlab script Monte_Carlo_Lokalizasyon_Algoritmasi.m
% Description: This script contains the implementation of the Monte Carlo Localization (MCL) algorithm, capable of estimating the unknown initial position of a mobile robot in an environment containing landmarks.
% *********************************************************************************************************************************************
% ***** The codes of the MCL algorithm were developed based on the information provided in Probabilistic Robotics authored by Thrun et al.*****
% *********************************************************************************************************************************************



% ***ÖNEMLİ UYARI!!!****
% Başlangıçta, parçacıklar haritanın tamamına rastgele atanmaktadır ve
% parçacık sayısınıa göre parçacıkların robotun gerçek konumunun etrafına
% düşme olasılıkları değişmektedir. Eğer başlangıç anında düzgün bir atama
% olmazsa simülasyonu durdurup tekrar başlatınız!

% Algoritmanın gerçek çalışma hızını tespit edebilmek için tüm Figure'ler
% kapatılmıştır. Figure'ler, başlarındaki parantez kaldırılarak açılabilir.

clc
clear all

%format long
bearing_Noise = 0.07; % Bearing açı gürültüsünün standart sapması (birimi: radyan)
olcum_Noise= 0.03; % Landmarktan alınan mesafe ölçüm gürültüsünün standart sapması (birimi: metre)


menzil = 0.35; % LiDAR'ın ölçüm menzili (birimi: metre)
N = 1000; % Parçacık sayısı

px = zeros(1,N); % Parçacıkların x ekseni üzerindeki konumlarınının yer aldığı dizi
py = zeros(1,N); % Parçacıkların y ekseni üzerindeki konumlarınının yer aldığı dizi
pteta = zeros(1,N); % Parçacıkların açılarının yer aldığı dizi

p1x = zeros(1,N);
p1y = zeros(1,N);
p1teta = zeros(1,N);
index = 1;

% Robot hareketinde kullanılan hata parametreleri
alfa1=0.01;
alfa2=0.01;
alfa3=0.01;
alfa4=0.01;
alfa5=0.001;
alfa6=0.001;

harita_x = 2; % haritanın x eksenindeki toplam uzunluğu (birimi: metre)
harita_y = 1.5; % haritanın y eksenindeki toplam uzunluğu (birimi: metre)

Landmarks = [[0.2,1];[0.35,0.6];[0.5,1.1];[0.6,0.7];[0.7,1.1];[0.75,0.55];[0.85,1.1];[1,0.5];[1.1,1];[1.4,1];[1.8,0.5];[1.8,0.88]]; % landmarkların bilinen konumları
Landmarkp=0;

C=[1,0,0;0,1,0;0,0,1;1,0,1];

d_robot = 0.06; % robot yön çubuğunun uzunluğu
d_parcacik = 0.03; % parçacık yön çubuğunun uzunluğu

Bearing = 0 ;
Landmark = 0;
hata = 0;
m = 0;

[px py pteta] = baslangic(harita_x,harita_y,N); % Başlangıç anında parçacıklar, haritanın tamamına rastgele uniform dağılımla dağıtılıyor
W(1:N) = (1/N); % Başlangıç anında robotun gerçek konumu bilinmediğinden, tüm parçacıkların robotun konumunda bulunma olasılıkları eşit ve 1/N olarak kabul edilir
W_aci(1:N) = (1/N);

cx = 0.40;  % Robotun başlangıç anında x ekseni üzerindeki gerçek konumu (metre)
cy = 0.78; % Robotun başlangıç anında y ekseni üzerindeki gerçek konumu (metre)
cteta = 0; % Robotun başlangıç anındaki gerçek açı değeri (radyan)

a  = 10+5*pi;
b  = 10-5*pi;
R  = 0.03; % Diferansiyel sürüş modeline sahip mobil robotun arka tekerleklerinin yarı çapı (birimi: metre)
d  = 0.3;  % İki tekerlek arasındaki mesafe (birimi: metre)
dt = 0.1;  % Örnekleme zamanı (birimi: saniye)

% Mobil robotun sağ ve sol tekerleklerinin dönme hızları (birimi: rad/sn)
wr = [ 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 a a a a a  10 10 10 10 10 10 10 10 10 10   b b b b b 10 10 10 10  b b b b b 10 10 10 10 10 10 10 10 10 10 10 a a a a a 10 10 10 10 10 10 10 10 10 10 10 10 10];
wl = [ 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 b b b b b  10 10 10 10 10 10 10 10 10 10   a a a a a 10 10 10 10  a a a a a 10 10 10 10 10 10 10 10 10 10 10 b b b b b 10 10 10 10 10 10 10 10 10 10 10 10 10];


adim_sayisi = length(wr);
Xr=zeros(adim_sayisi,3);
Pr=zeros(adim_sayisi,3);
durum = 0;
sayi = 1;
for k1 = 1:adim_sayisi
    tic
    Wxk = W;
    
    x1 = d_robot*cos(cteta) + cx;
    y1 = d_robot*sin(cteta) + cy;
    Xr(k1,:) = [cx cy cteta];

    % figure 1'i açmak için aşağıdaki parantezi kaldırınız.

    %{
    figure(1)
    clf
    plot(Xr(1:k1,1),Xr(1:k1,2),'g--','Linewidth',2);
    hold on
    [m1 n1] = size(Landmarks);
    for t = 1:m1
    plot(Landmarks(t,1),Landmarks(t,2),'b.','markersize',20)
    hold on
    scatter(Landmarks(t,1),Landmarks(t,2),200,[C(3,:)]);
    end
    
    if(Landmark(1,1) ~= 0)
    [m n] = size(Landmark);
    for t=1:m
    scatter(Landmark(t,1),Landmark(t,2),200,[C(4,:)]);
    end
    end
    

    plot(cx,cy,'g.','markersize',40);
    hold on
    line([cx x1],[cy y1],'Color','k','Linewidth',1.5);
    tx = 0:0.01*pi:2*pi;
    hold on
    plot(cx+(menzil)*sin(tx),cy+(menzil)*cos(tx),'r')
    
  
    for i = 1:N
    plot(px(i),py(i),'r.','markersize',10);
    hold on
     x1 = d_parcacik*cos(pteta(i)) + px(i);
     y1 = d_parcacik*sin(pteta(i)) + py(i);
     line([px(i) x1],[py(i) y1],'Color','k');
    
    end

     if(Landmark ~= 0)
        [m n] = size(Landmark);
        for i=1:m
            lanx = Landmark(i,1)+0.01*randn;
            lany = Landmark(i,2)+0.01*randn;
            line([cx lanx],[cy lany],'Color','g','Linewidth',0.5);
            hold on
            plot(lanx,lany,'y.','markersize',8);
        end
     end

   axis([0 harita_x 0 harita_y])
   
   title(['\color{magenta}Adım Sayısı = ',num2str(k1-1)],['\color{magenta}Tespit Edilen Landmark Sayısı = ',num2str(m)]);
   xlabel('X [m]');
   ylabel('Y [m]');
    %}
    Nx = length(px);
    
    Ns = 50;
    
    xx=0:1/(Ns):harita_x;
    yy=0:1/(Ns):harita_x;
   
    [Ppx Ppy]= meshgrid(xx,yy);
    Nn = length(xx);
    
    Wxy = zeros(Nn,Nn);
    
    for ij =1:Nx
        
        ix = round(px(ij)*Ns);
        iy = round(py(ij)*Ns);
        
        if iy == 0
            iy=1;
        end
        
        if ix == 0
            ix=1;
        end
        
            Wxy(iy,ix) = Wxk(ij);
        
    end

    % figure 2'yi açmak için aşağıdaki parantezi kaldırınız.

    %{
    figure(2)
    clf
   
    surf(Ppx,Ppy,Wxy)
    colormap jet
     title("Ağırlıklı Parçacıkların Dağılımı: Olasılıksal Dağılım");
     xlabel('X [m]');
     ylabel('Y [m]');
     zlabel('Parçacıkların Normalize Ağırlıkları');
     hold on
    axis([0 harita_x 0 harita_y])
    pause(0.001)
    %}
   
    
   [X Y O v w] = robot_hareket(wr(k1), wl(k1), R, cx, cy, cteta, dt, d, harita_x, harita_y); % robot hareket ediyor
    cx = X;
    cy = Y;
    cteta = O;


    %{
    if k1 == 32
        cx = cx+0.08*randn;
        cy = cy+0.08*randn;
        cteta = cteta + 0.01*pi*randn;
    end

    if k1 == 50
        cx = cx+0.08*randn;
        cy = cy+0.08*randn;
        cteta = cteta + 0.01*pi*randn;
    end

    if k1 == 64
        cx = cx+0.08*randn;
        cy = cy+0.08*randn;
        cteta = cteta + 0.01*pi*randn;
    end
    %}

    

    [X Y O] = parcacik_hareket(v,w,px,py,pteta,alfa1,alfa2,alfa3,alfa4,alfa5,alfa6,dt,N,harita_x,harita_y); % parçacıklar hareket ediyor
     
    px = X;
    py= Y;
    pteta=O;
    
    [Landmark Bearing] = landmark_cikarimi(cx, cy, cteta, Landmarks, menzil); % Robot üzerindeki mesafe algılayıcısının (LiDAR) menzili içindeki landmarklar (yer işareti) bir senaryoya göre çıkarılıyor.  
    
    [m1 n1] =size(Landmark);
    
    if(Landmark ~= 0)
    [Z] = landmark_olcum(Landmark, cx, cy, olcum_Noise); % gürültülü landmark ölçümleri alınıyor
    
    for i = 1:N 
    
    [Landmarkp Bearingp] = landmark_cikarimi_parcacik(px(i), py(i), pteta(i), Landmarks, menzil); % her bir parçacık için menzildeki landmarklar belirleniyior ve bu landmarklardan alınan bearing açılarıyla birlikte kaydediliyor.
     if(Landmarkp == 0)
         m2 = 0;
     else
    [m2 n2] =size(Landmarkp);
     end

    
    
    if(k1<15)  
        if(m1~=m2)
        W(i) = 0;
        W_aci(i)=0;
        else
       [Wi] = olcum_agirlik(Landmarkp, Z, px(i), py(i), olcum_Noise); % parçacıkların önem ağırlıkları hesaplanıyor
       W(i) = Wi;
      [W1i] = yonelim_agirlik(Bearing, Bearingp, bearing_Noise);
      W_aci(i) = W1i;
        end
    else
        if(m1~=m2)
        W(i) = 0;
        W_aci(i)=0;
        else
        [Wt] = birlesik_agirlik(Landmarkp, Z, px(i), py(i), olcum_Noise, Bearing, Bearingp, bearing_Noise); % parçacıkların önem ağırlıkları hesaplanıyor
        W(i) = Wt;
        end
    end
    
    end
  
    if(k1<15) 
    [px py W] = low_variance_resampler(px,py,W,N); % low variance yeniden örnekleme algoritması ile parçacıkların önem ağırlıklarına göre yeniden örneklemesi yapılıyor
    [pteta] = aci_resampler(W_aci,pteta,N);
    else
    [px py pteta W] = low_variance_resampler2(px,py,pteta,W,N); % low variance yeniden örnekleme algoritması ile parçacıkların önem ağırlıklarına göre yeniden örneklemesi yapılıyor
    end
     
    end

   
   
   % MCL algoritmasının konum tahmini yapılıyor:
   parcacikx = W*(px'); % Parçacıkların x ekseni üzerindeki ağırlıklı ortalaması
   parcaciky = W*(py'); % Parçacıkların y ekseni üzerindeki ağırlıklı ortalaması
   sonuc = 1;

   for ik =1:N
       if(pteta(ik)>pi)
          pteta(ik) = pteta(ik)-2*pi;
       end

       hata_p = sqrt((cx-px(ik))^2 + (cy-py(ik))^2);

       %Merkezi robotun bulunduğu konum, yarıçapı ise 0.2 metre olan çemberin içine parçacıkların tamamının yakınsayıp yakınsamadığını belirleyen yöntem:

       if hata_p>=0.2
           
           anahtar = 0;
       else
           anahtar = 1;
       end

       sonuc = sonuc*anahtar;
   end

   if(durum == 0)
   
       if sonuc == 0
          sayi = sayi+1;
       else
          durum = 1;
       end
   end
   
    parcacikteta = W*(pteta'); % Yönelim açısı tahmini
   Pr(k1,:) = [parcacikx parcaciky parcacikteta];
   hata(k1) = sqrt((cx-parcacikx)^2 + (cy-parcaciky)^2); % hata hesaplanıyor
   
   aci = (cteta- parcacikteta)*(180/pi);
    
   
     if(aci>=360)
        aci = (aci-360);
        
    end
    if(aci<=-360)
        aci = (aci+360);
        
    end
    
     if(aci>=360)
        aci = (aci-360);
        
    end
    if(aci<=-360)
        aci = (aci+360);
        
    end
    
     
    if(aci>=180)
        aci = (aci-360);
        
    end
    
    if(aci<=-180)
         aci = (360 +aci);
        
    end
   


   aci_hata(k1) = abs(aci);
    
    zaman(k1) = toc;
end

N = length(hata);

RMS_2B_hata = sqrt(sum(hata.*hata)/N) % birimi:[metre]

RMS_yonelim_hata = sqrt(sum(aci_hata.*aci_hata)/N) % birimi:[derece]

Ort_2B_hata = mean(hata) % birimi:[metre]

Std_dev = sqrt(var(hata)) % birimi:[metre]

Ort_Mutlak_Aci_hata = mean(aci_hata) % birimi:[derece]

ort_zaman = mean(zaman)*1000 % birimi:[milisaniye]

Yakinsama_adimi = sayi

%Sonucları .csv uzantılı dosyaya kaydetme:
T1 = table({'2B_Pozisyon_hata_dizisi: '; 'Yonelim_Mutlak_Hata_Dizisi: '}, [hata;aci_hata],'VariableNames', {'Deney Sonucu','  Deger'});
T2 = table({'Konum_RMS_Hata: ';'Yonelim_RMS_Hata: ';'Konum_Mean_Hata: ';'Yonelim_Mean_Hata: ';'Algoritmanin_ortalma_calisma_zamani [ms/adim]: ';'Parcaciklarin_tamaminin_yakinsadigi_adim_sayisi: '},[RMS_2B_hata;RMS_yonelim_hata;Ort_2B_hata;Ort_Mutlak_Aci_hata;ort_zaman;Yakinsama_adimi],'VariableNames', {'Deney Sonucu','   Deger'});
writetable(T1,'deney_sonuclari.csv');
writetable(T2, 'deney_sonuclari.csv', 'WriteMode', 'append');

% figure 4'ü açmak için aşağıdaki parantezi kaldırınız.

%{

for i=1:adim_sayisi
    
    cx = Xr(i,1);
    cy = Xr(i,2);
    cteta =Xr(i,3);
    px= Pr(i,1);
    py = Pr(i,2);
    pteta = Pr(i,3);
    
    p1 = d_robot*cos(pteta) + px;
    p2 = d_robot*sin(pteta) + py;
    
    x1 = d_robot*cos(cteta) + cx;
    y1 = d_robot*sin(cteta) + cy;
    
    figure(4)
    clf
    plot(Xr(1:i,1),Xr(1:i,2),'g--','Linewidth',2);
    hold on
    plot(Pr(1:i,1),Pr(1:i,2),'r-.','Linewidth',1.5);
    hold on
    [m1 n1] = size(Landmarks);
    for t = 1:m1
    plot(Landmarks(t,1),Landmarks(t,2),'b.','markersize',20)
    hold on
    end
    plot(cx,cy,'g.','markersize',40);
    hold on
    line([cx x1],[cy y1],'Color','k','Linewidth',1.5);
    hold on
    plot(px,py,'r.','markersize',40);
    hold on
    line([px p1],[py p2],'Color','k','Linewidth',1.5);
    hold on
    xlabel('X [m]');
    ylabel('Y[m]');
    title(['\color{red}Kırmızı: Konum Tahmini',', \color{green}Yeşil: Gerçek Konum'])
    axis([0 harita_x 0 harita_y])
    pause(0.01)
    
end

%}

%<===========FONKSİYONLAR===============>

function [px py pteta] = baslangic(harita_x,harita_y,N)

px = rand(1,N)*harita_x;
py = rand(1,N)*harita_y;
pteta = rand(1,N)*2*pi;

end


function [X Y O v w] = robot_hareket(wr, wl, R, cx, cy, cteta, dt, d, harita_x, harita_y) % robot ilerlemesi

    v= R*(wr+wl)/2;
    w = R*(wr-wl)/d;

    if w==0
        w = pi*10^(-7);
    end

    fark = v/w;
    X = cx - (fark).*sin(cteta) + (fark).*sin(cteta + dt*w);
    Y = cy + (fark).*cos(cteta) - (fark).*cos(cteta + dt*w);
    O = cteta + w*dt;
    
    X = mod(X,harita_x);
    Y = mod(Y,harita_y);
    O = mod(O,2*pi);
    
end


function [X Y O] = parcacik_hareket(v,w,px,py,pteta,alfa1,alfa2,alfa3,alfa4,alfa5,alfa6,dt,N,harita_x,harita_y) % parçacık ilerlemesi

 
E1 = zeros(12,N);
E2 = zeros(12,N);
E3 = zeros(12,N);

A1 = zeros(1,N);
A2 = zeros(1,N);
A3 = zeros(1,N);

b1= sqrt(alfa1*abs(v^2) + alfa2*abs(w^2));
b2= sqrt(alfa3*abs(v^2) + alfa4*abs(w^2));
b3= sqrt(alfa5*abs(v^2) + alfa6*abs(w^2));

for j = 1:12
    
R1 = 1-2*rand(1,N);
E1(j,:) = R1;

R2 = 1-2*rand(1,N);
E2(j,:) = R2;

R3 = 1-2*rand(1,N);
E3(j,:) = R3;

end

for i =1:N
    
    A1(i) = (b1/6)*(sum(E1(:,i)));
    A2(i) = (b2/6)*sum(E2(:,i));
    A3(i) = (b3/6)*sum(E3(:,i));
end

vn = v + A1;
wn = w + A2;
gamma = A3;
 
fark = vn./wn ;
 X = px(1:N) - (fark).*sin(pteta(1:N)) + (fark).*sin(pteta(1:N) + dt*wn) + normrnd(0,0.012,[1,N]);
 Y = (py(1:N) + (fark).*cos(pteta(1:N)) - (fark).*cos(pteta(1:N) + dt*wn)) + normrnd(0,0.012,[1,N]);
 O = pteta(1:N) + wn*dt + gamma*dt;


 X = mod(X,harita_x);
 Y = mod(Y,harita_y);
 O = mod(O,2*pi);

end


function [Landmark Bearing] = landmark_cikarimi(cx, cy, cteta, Landmarks, menzil)
Landmark = 0;
Bearing = 0;
[m n] = size(Landmarks);
k=1;
    for j = 1:m
        dist = sqrt((cx-Landmarks(j,1)).^2 + (cy-Landmarks(j,2)).^2);
       if(dist<=menzil)
           Landmark(k,1) = Landmarks(j,1);
           Landmark(k,2) = Landmarks(j,2);
           Bearing(k) = atan2(Landmark(k,2)-cy,Landmark(k,1)-cx);
           if Bearing(k)>pi
               Bearing(k) = Bearing(k)-2*pi;
           end
           if cteta>pi
               cteta = cteta-2*pi;
           end
           Bearing(k)=  Bearing(k) - cteta;
           k=k+1;
       end
    end

end



function [Landmarkp Bearingp] = landmark_cikarimi_parcacik(px, py, pteta, Landmarks, menzil)
Landmarkp = 0;
Bearingp = 0;
[m n] = size(Landmarks);

k=1;

for j = 1:m
        dist = sqrt((px-Landmarks(j,1)).^2 + (py-Landmarks(j,2)).^2);
       if(dist<=menzil)
           Landmarkp(k,1) = Landmarks(j,1);
           Landmarkp(k,2) = Landmarks(j,2);
           Bearingp(k) = atan2(Landmarkp(k,2)-py,Landmarkp(k,1)-px);
           if Bearingp(k)>pi
               Bearingp(k) = Bearingp(k)-2*pi;
           end
            if pteta>pi
               pteta = pteta-2*pi;
           end
           Bearingp(k)=  Bearingp(k) - pteta;
           k=k+1;
       end
end

end

function [Z] = landmark_olcum(Landmark, cx, cy, olcum_Noise)


[m n] = size(Landmark);


    
    for j = 1:m
        dist = sqrt((cx-Landmark(j,1)).^2 + (cy-Landmark(j,2)).^2);
        dist = dist + normrnd(0,olcum_Noise);
        fark(j) = dist;
    end
    Z = fark;

end


function [Wi] = olcum_agirlik(Landmarkp, Z, pcx, pcy, olcum_Noise)


[m n] = size(Landmarkp);

    prob=1;
    for j = 1:m
        dist = sqrt((pcx-Landmarkp(j,1)).^2 + (pcy-Landmarkp(j,2)).^2);
       k=0.5*((dist-Z(j))^2/(olcum_Noise^2));
       g = (exp(-k))/(sqrt(2*pi*(olcum_Noise^2)));
       prob = prob*g;
    end
    Wi=prob;
    if(Wi<=0.0)
        Wi=0;
    end

end


function [W1] = yonelim_agirlik(Bearing, Bearingp,bearing_Noise)

    n = length(Bearing);
    prob =1;
   
    for i =1:n
    aci = (Bearing(i)-Bearingp(i))*(180/pi); 
    
    if(aci>=360)
        aci = (aci-360);
        
    end
    if(aci<=-360)
        aci = (aci+360);
        
    end
    
     if(aci>=360)
        aci = (aci-360);
        
    end
    if(aci<=-360)
        aci = (aci+360);
        
    end
    
     
    if(aci>=180)
        aci = (aci-360);
        
    end
    
    if(aci<=-180)
         aci = (aci+360);
        
    end

    dist = aci*(pi/180);

     k=0.5*((dist)^2/(bearing_Noise^2));
     g = (exp(-k))/(sqrt(2*pi*(bearing_Noise^2)));
     prob = prob*g;
    end
     W1 = prob;
     
    if(W1<=0)
        W1=0;
    end
end


function [Wt] = birlesik_agirlik(Landmarkp, Z, pcx, pcy, olcum_Noise, Bearing, Bearingp,bearing_Noise)

    n = length(Bearing);
    prob =1;
    Q = [olcum_Noise^2 0; 0 bearing_Noise^2];
   
    for i =1:n
    aci = (Bearing(i)-Bearingp(i))*(180/pi); 
    
    if(aci>=360)
        aci = (aci-360);
        
    end
    if(aci<=-360)
        aci = (aci+360);
        
    end
    
     if(aci>=360)
        aci = (aci-360);
        
    end
    if(aci<=-360)
        aci = (aci+360);
        
    end
    
     
    if(aci>=180)
        aci = (aci-360);
        
    end
    
    if(aci<=-180)
         aci = (aci+360);
        
    end

    dist_aci = aci*(pi/180);
    dist_landmark = sqrt((pcx-Landmarkp(i,1)).^2 + (pcy-Landmarkp(i,2)).^2);
    dist_olcum = Z(i)-dist_landmark;
    zv = [dist_olcum;dist_aci];
     g = ((det(2*pi*Q))^(-0.5))*exp(-0.5*(zv')*(inv(Q))*(zv));
     prob = prob*g;
    end
     Wt = prob;
     
    if(Wt<=0)
        Wt=0;
    end
end


function [px py Weight] = low_variance_resampler(px,py,Weight,Np)

toplam = sum(Weight);
Weight = Weight/toplam;
wk(1:Np) = 1/Np;
wa = Weight(1);
r = (rand)/Np;
iw = 1;

for mw = 1:Np 
  
    
    U = r + (mw-1)/(Np);
 
     
     while U > wa                
             iw = iw + 1;
             wa = wa + Weight(iw);
     end

     psx(mw) = px(iw);
     psy(mw) = py(iw);
     wk(mw) = Weight(iw);
    
end

px = psx;
py = psy;
Weight = wk;
Weight = Weight/sum(Weight);

end

function [px py pteta Weight] = low_variance_resampler2(px,py,pteta,Weight,Np)

toplam = sum(Weight);
Weight = Weight/toplam;
wk(1:Np) = 1/Np;
wa = Weight(1);
r = (rand)/Np;
iw = 1;

for mw = 1:Np 
  
    
    U = r + (mw-1)/(Np);
 
     
     while U > wa                
             iw = iw + 1;
             wa = wa + Weight(iw);
     end

     psx(mw) = px(iw);
     psy(mw) = py(iw);
     psteta(mw) = pteta(iw);
     wk(mw) = Weight(iw);
    
end

px = psx;
py = psy;
pteta = psteta;
Weight = wk;
Weight = Weight/sum(Weight);

end


function [pteta] = aci_resampler(W_aci,pteta,N)

toplam = sum(W_aci);
W_aci = W_aci/toplam;
cumwt = cumsum(W_aci);
p_teta(1:N) = 0;

     for kx = 1:N
   
         swt = rand; 
         index = find( cumwt>= swt,1,'first');
         if(index<=0)
            index =1;
         end
   
  
         p_teta(kx) = pteta(index);
     end

pteta = p_teta;

end


