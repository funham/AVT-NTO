import facility_api as fac

def moveToHoarder(hoarder: int):

    if hoarder == 0:
        ToFirstHoarder()
    elif hoarder == 1:
        ToSecondHoarder()
    elif hoarder == 2:
        ToThirdHoarder()
    else:  # hoarder == 3
        ToFourthHoarder()