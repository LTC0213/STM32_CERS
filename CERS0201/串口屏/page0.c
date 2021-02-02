va1.val=gross.val/10
cov va1.val,tempstr1.txt,0
va1.val=gross.val%10
if(va1.val<0)
{
  va1.val=0-va1.val
}
cov va1.val,tempstr2.txt,1
if(gross.val>=0)
{
  t0.txt=tempstr1.txt+"."+tempstr2.txt
}else if(gross.val>-10)
{
  t0.txt="-"+tempstr1.txt+"."+tempstr2.txt
}else
{
  t0.txt=tempstr1.txt+"."+tempstr2.txt
}
va1.val=net.val/10
cov va1.val,tempstr1.txt,0
va1.val=net.val%10
if(va1.val<0)
{
  va1.val=0-va1.val
}
cov va1.val,tempstr2.txt,1
if(net.val>=0)
{
  t2.txt=tempstr1.txt+"."+tempstr2.txt
}else if(net.val>-10)
{
  t2.txt="-"+tempstr1.txt+"."+tempstr2.txt
}else
{
  t2.txt=tempstr1.txt+"."+tempstr2.txt
}
