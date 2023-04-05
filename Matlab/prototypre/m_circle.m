function circles = m_circle (x,y,r)
%CIRCLE  이 함수의 요약 설명 위치
%   자세한 설명 위치
hold on
th = 0:pi/150:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;
circles = plot(x_circle, y_circle);

hold off
end

