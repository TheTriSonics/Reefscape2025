from datetime import datetime, timedelta

def get_market_holidays_2025():
    """Returns a list of market holidays for 2025."""
    return [
        datetime(2025, 1, 1),   # New Year's Day
        datetime(2025, 1, 20),  # Martin Luther King Jr. Day
        datetime(2025, 2, 17),  # Presidents Day
        datetime(2025, 4, 18),  # Good Friday
        datetime(2025, 5, 26),  # Memorial Day
        datetime(2025, 6, 19),  # Juneteenth
        datetime(2025, 7, 4),   # Independence Day
        datetime(2025, 9, 1),   # Labor Day
        datetime(2025, 11, 27), # Thanksgiving Day
        datetime(2025, 12, 25), # Christmas Day
    ]

def is_trading_day(date, holidays):
    """
    Check if a given date is a trading day.
    Returns False for weekends and holidays.
    """
    # Check if it's a weekend
    if date.weekday() >= 5:
        return False
    
    # Check if it's a holiday
    return date not in holidays

def count_full_trading_weeks(start_date, end_date):
    """
    Count weeks where the market was open all five weekdays.
    """
    holidays = get_market_holidays_2025()
    current_date = start_date
    full_weeks = 0
    
    while current_date <= end_date:
        # Find the Monday of the current week
        monday = current_date - timedelta(days=current_date.weekday())
        
        # Check all days in the week
        week_days = [monday + timedelta(days=i) for i in range(5)]
        all_days_trading = all(is_trading_day(day, holidays) for day in week_days)
        
        if all_days_trading:
            full_weeks += 1
        
        # Move to next week's Monday
        current_date = monday + timedelta(days=7)
    
    return full_weeks

def main():
    start_date = datetime(2025, 1, 1)
    end_date = datetime(2025, 2, 4)  # Current date
    
    full_weeks = count_full_trading_weeks(start_date, end_date)
    print(f"Number of full trading weeks from {start_date.date()} to {end_date.date()}: {full_weeks}")
    
    # Optional: Print out the weeks for verification
    current_date = start_date
    while current_date <= end_date:
        monday = current_date - timedelta(days=current_date.weekday())
        week_days = [monday + timedelta(days=i) for i in range(5)]
        
        holidays = get_market_holidays_2025()
        all_days_trading = all(is_trading_day(day, holidays) for day in week_days)
        
        if all_days_trading:
            print(f"Full trading week: {monday.date()} to {(monday + timedelta(days=4)).date()}")
        else:
            non_trading_days = [day.date() for day in week_days if not is_trading_day(day, holidays)]
            print(f"Partial week: {monday.date()} to {(monday + timedelta(days=4)).date()}")
            print(f"  Non-trading days: {non_trading_days}")
            
        current_date = monday + timedelta(days=7)

if __name__ == "__main__":
    main()
