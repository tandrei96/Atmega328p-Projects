#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>


int main(){
    char buffer[200];
    FILE* fpFile1;
    FILE* fpFile2;

    unsigned int i,j=0;
    fpFile1=fopen("output.txt","r");
    fpFile2=fopen("result.txt","w+");

    if(fpFile1==NULL){
        perror("Error detected: \n");
    }
    else if(fpFile2==NULL) {
        perror("Error detected: \n");
    }
    else{
        while(1){
                i=fgetc(fpFile1);
                j=i;
            if(i==EOF){
                break;
            }
            /*Aveam multe caractere de terminal care nu faceau parte din codul ascii. Aici i-am zis sa-mi sorteze doar caracterele de la 32 la 126 ca alea fac parte din codul ASCII. a mers pentru ca j e integer*/
            else if(j>32 && j<126) {
                fputc((int)j,fpFile2);
                fputs("\n",fpFile2);
                printf("%c",j);
            }
            else{
                //
            }
        }
        fclose(fpFile1);
        fclose(fpFile2);
    }

    return 0;
}