#https://pypi.org/project/newportxps/

from newportxps import NewportXPS

def main():

    IP = '10.1.137.137'
    #This only works with administrator accounts on the XPS for some reason. 
    global xps 
    xps = NewportXPS(IP, username='PyControl', password='labview2python',port=5001)
    # lists valid group names
    print(xps.status_report())
    xps.read_systemini()   # or however the wrapper exposes it
    print(xps.groups)

if __name__ == "__main__":
    main()
      


xps.disconnect()