#pragma once

#include <vector>

#include <units/angle.h>
#include <units/length.h>
#include <frc/trajectory/TrapezoidProfile.h>

// Use unit values as types.
template<class XType, class YType> class Constraint 
{
public:
    using XType_t = units::unit_t<XType>;
    using YType_t = units::unit_t<YType>;
    struct Point { 
        XType_t x; 
        YType_t y; 
    };

    Constraint( ) {}
    void AddConstraint( Point p, bool allowLessThan );
    std::vector<Point> FindPath( Point start, Point end );

private:
    bool FindAnyBreaks( std::vector<Point> &path );

    using Slope = units::compound_unit<YType, units::inverse<XType>>;
    using Slope_t = units::unit_t<Slope>;
    std::vector<std::pair<Point,bool>> pts;
};

template<class Distance> class TrapezoidProfileByTime
{
public:
    using Distance_t = units::unit_t<Distance>;
    using Velocity_t = frc::TrapezoidProfile<Distance>::Velocity_t;
    using Acceleration_t = frc::TrapezoidProfile<Distance>::Acceleration_t;
    using State = frc::TrapezoidProfile<Distance>::State;
    using Constraints = frc::TrapezoidProfile<Distance>::Constraints;

    TrapezoidProfileByTime( units::second_t TotalTime, Distance_t TotalDistance, Constraints constraints );

    State Calculate( units::second_t delta_t, State current );
    units::second_t TotalTime() const { return m_Profile.TotalTime(); }
private:
    frc::TrapezoidProfile<Distance> m_Profile;
    units::second_t m_TotalTime;
    Distance_t m_TotalDist;
    Velocity_t m_MaxVelocity;
    Acceleration_t m_MaxAccel;
};

template<class Dist1, class Dist2> class SyncTrapezoidProfiles
{
public:
    using State1 = frc::TrapezoidProfile<Dist1>::State;
    using State2 = frc::TrapezoidProfile<Dist2>::State;
    struct SyncState { 
        State1 state1; 
        State2 state2; 
    };
    SyncTrapezoidProfiles( frc::TrapezoidProfile<Dist1>::Constraint c1, frc::TrapezoidProfile<Dist2>::Constraint c2, State1 goal1, State2 goal2 );

    SyncState Calculate( units::second_t delta_t, SyncState current );
    units::second_t TotalTime() const { return m_Profile1.TotalTime(); }

private:
    frc::TrapezoidProfile<Dist1> m_Profile1;
    frc::TrapezoidProfile<Dist2> m_Profile2;
    State1 m_goal1;
    State2 m_goal2;
};


template<class XType, class YType> void Constraint<XType,YType>::AddConstraint( Point p, bool allowLessThan )
{
    if( pts.size() == 0 ) {
        pts.push_back( std::pair(p,allowLessThan ) );
    }

        // Make sure our constraint points stay in increasing X order.
    XType_t lowerx{XType_t( -9999999 )};
    for( auto piter = pts.begin(); piter != pts.end(); piter++ ) {
        if( p.x > lowerx && p.x < piter->first.x ) {
            pts.insert( piter, {p,allowLessThan} );
            break;
        } else if( p.x > piter->first.x && piter == pts.end()-1 ) {
            pts.push_back( {p,allowLessThan} );
            break;
        }
        lowerx = piter->first.x;
    }
}  

template<class XType, class YType> 
std::vector<typename Constraint<XType,YType>::Point> Constraint<XType,YType>::FindPath( Point start, Point end ) 
{
    bool reverse = false;
    Point beginPt, endPt;
    if( start.x > end.x ) {
        beginPt = end;
        endPt = start;
        reverse = true;
    } else {
        beginPt = start;
        endPt = end;
    }

    std::vector<Point> path;

        // Start with a straight line path.
    path.push_back( beginPt );
    path.push_back( endPt );

    bool path_broken = true;

        // If the path is broken by a constraint then we
        // must start over with the broken path segments because
        // the slopes change.  Do this until the path is not
        // broken anymore.
    while( path_broken ) {
        path_broken = FindAnyBreaks( path );
    }
 
    if( reverse ) {
        std::vector<Point> v_reverse = path;
        path = std::vector<Point>{};
        for( size_t i=v_reverse.size(); i>0; --i ) {
            path.push_back( v_reverse[i-1] );
        }
    }

    for( size_t i=0; i<path.size(); ++i ) {
        fmt::print( "Path Point #{} -- ({},{})\n", i, path[i].x, path[i].y );
    }
    return path;
}


template<class XType, class YType> 
bool Constraint<XType,YType>::FindAnyBreaks( std::vector<Point> &path )
{
        // Loop over all the line segments in the path.
    for( auto pptr=path.begin(); pptr<path.end()-1; ++pptr ) {
        Point beginPt = *pptr;
        Point endPt = *(pptr+1);
        XType_t run = endPt.x - beginPt.x;
        if( units::math::abs(run) < XType_t(0.01) ) {
            fmt::print( "Constraint::FindPath -- Slope is infinite!!\n" );
            return false;
        }

           // Find the slope and intercept.
        Slope_t slope = ( endPt.y - beginPt.y ) / run;
        YType_t intercept = endPt.y - slope * endPt.x;

        for( std::pair<Point,bool> p : pts ) {
            if( p.first.x > beginPt.x && p.first.x < endPt.x ) {
                // Constraint is within the start and end point values of this line segment.
                YType_t line_y = slope * p.first.x + intercept;
                if( ( p.first.y < line_y && p.second ) || ( p.first.y > line_y && !p.second ) ) {
                    // Constraint point is below the line and it is an allowLessThan point
                    // or constraint point is above the line and it is not an allowLessThan point.
                    // We break the path here by inserting the constraint point into the path.
                    // Return true so the entire path can be recomputed with the new line segment
                    // since breaking the path causes the slopes to change.
                    path.insert( pptr+1, p.first );
                    return true;
                }
            }
        }
    }
    return false;
}

template<class Distance>
TrapezoidProfileByTime<Distance>::TrapezoidProfileByTime( units::second_t TotalTime, Distance_t TotalDistance, Constraints constraints )
    : m_TotalTime{TotalTime}, m_TotalDist{TotalDistance}, m_MaxAccel{constraints.maxAcceleration}, m_Profile{constraints}
{
    /**
     * Use Quadratic Formula to determine the Maximum Velocity that will result in a TrapezoidProfile that covers the
     * given distance in the given total time.
     * 
     * For the quadratic:   ax^2 + bx + c = 0 
     * 
     *      a = -1.0 / MaxAccel
     *      b = TotalTime
     *      c = -1.0 * TotalDistance
     * 
     *      radical = b*b - 4*a*c
    */

    double a, b, c;

    a = -1.0 / m_MaxAccel.value();
    b = m_TotalTime.value();
    c = -1.0 * m_TotalDist.value();

    double radical = b*b - 4*a*c;
    if( fabs(radical) < 0.0001 ) {
        // Radical is zero which means the fastest time is the correct profile
        m_MaxVelocity = constraints.maxVelocity;
    } else if( radical < 0.0 ) {
        // Malformed means that the desired total time is shorter than the fastest profile time.
        // Use the fastest profile.
        fmt::print( "Malformed TrapezoidProfileByTime!!!   Complex number solutions!!!! radical={}\n", radical );
        m_MaxVelocity = constraints.maxVelocity;
    } else {
        // Slowed trapezoid profile case..

        double soln1 = ( -1.0 * b - sqrt( radical ) ) / (2*a);
        double soln2 = ( -1.0 * b + sqrt( radical ) ) / (2*a);

        if( soln1 > 0.0 && (soln1 * a * -1.0) < b/2 ) {
            m_MaxVelocity = Velocity_t( soln1 );
        } else if( soln2 > 0.0 && (soln2 * a * -1.0) < b/2 ) {
            m_MaxVelocity = Velocity_t( soln2 );
        } else {
            fmt::print( "No Valid solution TrapezoidProfileByTime!!!   V1={}, V2={}!!!!\n", soln1, soln2 );
            m_MaxVelocity = Velocity_t(0);
        }
    }

    m_Profile = frc::TrapezoidProfile<Distance>{ {m_MaxVelocity, m_MaxAccel} };
    if( m_MaxVelocity/m_MaxAccel > m_TotalTime / 2 ) {
        // Short move that doesn't reach max velocity
        fmt::print( "   MaxVelocity={}, t_A={}, t_C={}\n", m_MaxVelocity, m_TotalTime/2, 0_s );
    } else {
        fmt::print( "   MaxVelocity={}, t_A={}, t_C={}\n", m_MaxVelocity, m_MaxVelocity/ m_MaxAccel, m_TotalTime - (2* m_MaxVelocity/ m_MaxAccel) );
    }
}

template<class Distance>
TrapezoidProfileByTime<Distance>::State TrapezoidProfileByTime<Distance>::Calculate( units::second_t delta_t, State current )
{
    return m_Profile.Calculate( delta_t, current, {m_TotalDist, Velocity_t(0.0)} );
}
