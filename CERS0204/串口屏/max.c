va1.val=avg1.val/10
cov va1.val,tempstr1.txt,0
va1.val=avg1.val%10
if(va1.val<0)
{
  va1.val=0-va1.val
}
cov va1.val,tempstr2.txt,1
if(avg1.val>=0)
{
  t14.txt=tempstr1.txt+"."+tempstr2.txt
}else if(avg1.val>-10)
{
  t14.txt="-"+tempstr1.txt+"."+tempstr2.txt
}else
{
  t14.txt=tempstr1.txt+"."+tempstr2.txt
}
va1.val=avg2.val/10
cov va1.val,tempstr1.txt,0
va1.val=avg2.val%10
if(va1.val<0)
{
  va1.val=0-va1.val
}
cov va1.val,tempstr2.txt,1
if(avg2.val>=0)
{
  t15.txt=tempstr1.txt+"."+tempstr2.txt
}else if(avg2.val>-10)
{
  t15.txt="-"+tempstr1.txt+"."+tempstr2.txt
}else
{
  t15.txt=tempstr1.txt+"."+tempstr2.txt
}
va1.val=avg3.val/10
cov va1.val,tempstr1.txt,0
va1.val=avg3.val%10
if(va1.val<0)
{
  va1.val=0-va1.val
}
cov va1.val,tempstr2.txt,1
if(avg3.val>=0)
{
  t16.txt=tempstr1.txt+"."+tempstr2.txt
}else if(avg3.val>-10)
{
  t16.txt="-"+tempstr1.txt+"."+tempstr2.txt
}else
{
  t16.txt=tempstr1.txt+"."+tempstr2.txt
}
va1.val=max1.val/10
cov va1.val,tempstr1.txt,0
va1.val=max1.val%10
if(va1.val<0)
{
  va1.val=0-va1.val
}
cov va1.val,tempstr2.txt,1
if(max1.val>=0)
{
  t20.txt=tempstr1.txt+"."+tempstr2.txt
}else if(max1.val>-10)
{
  t20.txt="-"+tempstr1.txt+"."+tempstr2.txt
}else
{
  t20.txt=tempstr1.txt+"."+tempstr2.txt
}
va1.val=max2.val/10
cov va1.val,tempstr1.txt,0
va1.val=max2.val%10
if(va1.val<0)
{
  va1.val=0-va1.val
}
cov va1.val,tempstr2.txt,1
if(max2.val>=0)
{
  t21.txt=tempstr1.txt+"."+tempstr2.txt
}else if(max2.val>-10)
{
  t21.txt="-"+tempstr1.txt+"."+tempstr2.txt
}else
{
  t21.txt=tempstr1.txt+"."+tempstr2.txt
}
va1.val=max3.val/10
cov va1.val,tempstr1.txt,0
va1.val=max3.val%10
if(va1.val<0)
{
  va1.val=0-va1.val
}
cov va1.val,tempstr2.txt,1
if(max3.val>=0)
{
  t22.txt=tempstr1.txt+"."+tempstr2.txt
}else if(max3.val>-10)
{
  t22.txt="-"+tempstr1.txt+"."+tempstr2.txt
}else
{
  t22.txt=tempstr1.txt+"."+tempstr2.txt
}