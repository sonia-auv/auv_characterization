X = LINEAR_SPEED(:,1);
Y = Thrust;
d = 1:332;
X = sgolayfilt(X,20,57);

data = iddata(X, Y, T);



ge = etfe(data, 8);
gs = spa(data, 8);

bode(ge, gs)
