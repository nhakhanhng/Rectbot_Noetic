v_te=v_e*1000/t
        w_te=w_e*1000/t

        dphi =w_e
        phi=phi+ dphi

        dx=v_e * cos(phi)
        dy=v_e * sin(phi)

        x=x+ dx
        y=y+ dy
        print "x: ",x
        print "y: ",y

        if(phi>=6.28):
            phi=phi-6.28
        if(phi<=-6.28):
            phi=phi+6.28
        enc1old = enc1
        enc2old = enc2
        quakhu = rospy.Time.now(