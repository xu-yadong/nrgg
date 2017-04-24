SOURCE_ORG="../src"
INC_ORG="../include"

for i in $SOURCE_ORG/*.cpp;
do 
X2=$(pwd $i); 	
Y2="$X2/$i";
echo $Y2
FILENAME=$(basename $i)
g++ -I../include -Wall -c -fmessage-length=0 -std=c++11 -fPIC -MMD -MP -MF${FILENAME%.cpp}".d" -MT${FILENAME%.cpp}".d" -o ${FILENAME%.cpp}".o" $Y2
done

TEST="ar -r libnrgg.a";
for i in *.o;
do 
TEST="${TEST} $i";
done
$TEST

rm -R *.d
rm -R *.o
