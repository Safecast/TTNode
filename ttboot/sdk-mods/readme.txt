
//
// COPIED FROM VIDAR'S SERIAL_DFU SAMPLE
//
In components/libraries/bootloader/nrf_bootloader_app_start.c, within nrf_bootloader_app_start(), change ...
#ifdef BLE_STACK_SUPPORT_REQD
...to...
#if defined(BLE_STACK_SUPPORT_REQD) || defined(SERIAL_DFU_APP_REQUIRES_SD)
...twice.
//


//
// BUG THAT CAUSED INCORRECT MAIN_APPLICATION_START_ADDR TO BE RETURNED BY NRF_DFU_FIND_CACHE()
//
In components/libraries/bootloader/nrf_bootloader_info.h,
...change
#ifdef SOFTDEVICE_PRESENT
...to...
#if defined(SOFTDEVICE_PRESENT) || defined(SERIAL_DFU_APP_REQUIRES_SD)
...twice.
//


//
// SIGNIFICANT SCHEDULER REENTRANCY ISSUE THAT CAUSED MATERIAL CODE RESTRUCTURING
//
In components/libraries/scheduler/app_scheduler.c, within app_sched_execute(), change...

        event_handler(p_event_data, event_data_size);

        // Event processed, now it is safe to move the queue start index,
        // so the queue entry occupied by this event can be used to store
        // a next one.
        m_queue_start_index = next_index(m_queue_start_index);

…to this…

        // Fully dequeue this event, freeing up the queue entry
        m_queue_start_index = next_index(m_queue_start_index);

        // Call the handler with the entry data fully in stack locals.
        // This enables app_sched_event_put() or app_sched_execute()
        // to be safely called from within the event handler.
        event_handler(p_event_data, event_data_size);

//
