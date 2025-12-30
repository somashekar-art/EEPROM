`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Kaynes Semicon
// Engineer: RTL Design 

// Create Date: 22.12.2025 14:56:43
// Design Name: 
// Module Name: I2C_EEPROM_Controller
// Project Name: 
// Target Devices: EEPROM 
//////////////////////////////////////////////////////////////////////////////////

module I2C_EEPROM_Controller(
    input wire clk,
    input wire reset,
    input wire [6:0] addr,
    input wire [6:0] addr_1,
    input wire [6:0] read_addr,
    input wire [7:0] addr_byte,
    input wire [7:0] addr_byte_1,
    input wire [7:0] read_addr_byte,
    input wire [7:0] read_addr_byte_1,
    input wire [7:0] data_in,    
    input wire start,
    input wire read,
    input wire rw, 
    input wire rw_1,
    input wire read_rw,
    
    output wire i2c_sda_t,    
    output wire i2c_sda_o,  
    output wire i2c_sda_i,     
    inout wire i2c_sda,
    
    output reg i2c_scl,
    output reg [7:0] data_out,    
    output wire SDA_D,
    output wire [5:0] status_s
);
    
    reg phase_tick;    
    reg [4:0] state;    
    reg [7:0] shift_reg_in;
    reg [7:0] shift_reg_out;
    reg [20:0] bit_count;
    reg [15:0] counter;  
    reg [25:0] wait_counter; 
    reg sda_tristate;  
    reg sda_data;                              
    
    localparam FREQ           = 125,    // 400kHz I2C clock from 50MHz system clock
               START_HOLD     = 62,
               START_HOLE_ACK = 55,
               STOP_BIT       = 110,
               WAIT_5MS       = 5_00_000;  // 5ms wait for EEPROM write cycle
    
    localparam IDLE                   = 5'd0,
               START                  = 5'd1,             
               ADDRESS                = 5'd2,
               ADDR_ACK               = 5'd3,
               REG_ADDR               = 5'd4,
               REG_ADDR_ACK           = 5'd5,
               REG_ADDR_1             = 5'd6,
               REG_ADDR_ACK_1         = 5'd7,
               DATA                   = 5'd8,
               DATA_ACK               = 5'd9,
               STOP                   = 5'd10,
               WAIT                   = 5'd11,
               START_AGAIN            = 5'd12,
               ADDRESS_1              = 5'd13,
               ADDR_ACK_1             = 5'd14, 
               READ_REG_ADDR          = 5'd15,
               READ_REG_ADDR_ACK      = 5'd16,
               READ_REG_ADDR_1        = 5'd17,
               READ_REG_ADDR_ACK_1    = 5'd18,
               RE_START               = 5'd19,
               READ_ADDRESS_1         = 5'd20,
               READ_ADDR_ACK_1        = 5'd21,
               READ_DATA              = 5'd22,                                             
               READ_NACK              = 5'd23,
               STOP_1                 = 5'd24,
               STORE_STATUS_CPTURE_ID = 5'd25;
               
    // Bidirectional buffer for SDA line
    IOBUF #(
        .DRIVE(12),
        .IOSTANDARD("LVCMOS33"),
        .SLEW("SLOW")
    ) IOBUF_sda (
        .O(i2c_sda_i),     // Output from pad to FPGA
        .IO(i2c_sda),      // Bidirectional pad
        .I(i2c_sda_o),     // Input from FPGA to pad
        .T(i2c_sda_t)      // Tri-state control (1 = Hi-Z, 0 = Drive)
    );
                      
    wire i2c_active;
    assign i2c_active = (state != IDLE && state != STORE_STATUS_CPTURE_ID);   
    assign status_s = state;
    
    assign i2c_sda_t = sda_tristate;
    assign i2c_sda_o = sda_data;
    assign SDA_D     = i2c_sda_i;
    
        
            always @(posedge clk or negedge reset) begin
                if (!reset) begin
                    counter    <= 0;               
                    phase_tick <= 0;
                end else if (i2c_active) begin
                    if (counter == FREQ - 1) begin                
                        counter    <= 0;
                        phase_tick <= 1;
                    end else begin
                        counter <= counter + 1;
                        phase_tick <= 0;
                    end
                end else begin
                    counter    <= 0;
                    phase_tick <= 0;
                end
            end
            
    
    always @(posedge clk or negedge reset) begin
        if (!reset) begin 
            sda_tristate <= 0;  
            sda_data     <= 1;                        
            i2c_scl      <= 1;                             
            shift_reg_in <= 0;
            shift_reg_out <= 0;               
            data_out     <= 0;                          
            wait_counter <= 0;
            bit_count    <= 0;
            state        <= IDLE;
        end else begin
            case(state)
                
                IDLE: begin
                      i2c_scl      <= 1;     
                      sda_tristate <= 1;        
                      sda_data     <= 1;
                      bit_count    <= 0;
                      wait_counter <= 0;
                        if (start) begin                                                                                                              
                            sda_tristate <= 0;  
                            sda_data     <= 1;                            
                            i2c_scl      <= 1;                                                                                                                     
                            state        <= START;                        
                        end else if (read) begin
                            sda_tristate <= 0;  
                            sda_data     <= 1;                            
                            i2c_scl      <= 1;                                                                                                                     
                            state        <= START_AGAIN;                     
                        end
                    end
                              
                
                START: begin                                                                                                                            
                        bit_count     <= 0;
                        shift_reg_out <= {addr, rw};
                    
                        if (counter == START_HOLD) begin
                            sda_tristate <= 0;  
                            sda_data     <= 0;  
                            i2c_scl      <= 1;  
                        end
        
                        if (phase_tick) begin
                            i2c_scl <= ~i2c_scl;  
                            
                            if (i2c_scl == 1) begin  
                                state <= ADDRESS;
                            end
                        end                                                                                                                        
                    end  
                
               ADDRESS: begin                                                                                                                                                 
                        if (i2c_scl == 0 && counter == START_HOLD) begin                                              
                            sda_tristate <= 0;      
                            sda_data     <= shift_reg_out[7 - bit_count];                                   
                        end
                    
                        if (phase_tick) begin
                            i2c_scl <= ~i2c_scl;
                            
                            if (i2c_scl == 1) begin
                                if (bit_count == 7) begin                 
                                    bit_count <= 0;
                                    state <= ADDR_ACK;  
                                end else begin
                                    bit_count <= bit_count + 1;
                                end  
                            end
                        end
                    end                                                                                                                
                
                               
                ADDR_ACK: begin                   
                          if (i2c_scl == 0 && counter == START_HOLD) begin 
                                  sda_tristate <= 1;                                    
                          end
                    
                            if (phase_tick) begin 
                                i2c_scl <= ~i2c_scl;
                                
                                if (i2c_scl == 1) begin
                                    state <= REG_ADDR;
                                end
                            end
                        end
                
                
                
               REG_ADDR: begin   
                        shift_reg_out <= addr_byte;                                                                                                                                              
                        if (i2c_scl == 0 && counter == START_HOLD) begin                                              
                            sda_tristate <= 0;      
                            sda_data     <= shift_reg_out[7 - bit_count];                                   
                        end
                        
                        if (phase_tick) begin
                            i2c_scl <= ~i2c_scl;
                            
                            if (i2c_scl == 1) begin
                                if (bit_count == 7) begin                 
                                    bit_count <= 0;
                                    state <= REG_ADDR_ACK;  
                                end else begin
                                    bit_count <= bit_count + 1;
                                end  
                            end
                        end 
                    end  
                
                
                REG_ADDR_ACK: begin
                              if (i2c_scl == 0 && counter == START_HOLD) begin 
                                  sda_tristate <= 1;                                    
                              end
                    
                                if (phase_tick) begin 
                                    i2c_scl <= ~i2c_scl;
                                    
                                    if (i2c_scl == 1) begin
                                        state <= REG_ADDR_1;
                                    end
                                end
                            end   
                        
                
                REG_ADDR_1: begin  
                               shift_reg_out <= addr_byte_1;                                                                                                                                               
                            if (i2c_scl == 0 && counter == START_HOLD) begin                                              
                                sda_tristate <= 0;      
                                sda_data     <= shift_reg_out[7 - bit_count];                                   
                             end
                    
                            if (phase_tick) begin
                                i2c_scl <= ~i2c_scl;
                                
                                if (i2c_scl == 1) begin
                                    if (bit_count == 7) begin                 
                                        bit_count <= 0;
                                        state <= REG_ADDR_ACK_1;  
                                    end else begin
                                        bit_count <= bit_count + 1;
                                    end  
                                end
                            end
                        end  
                        
                
                REG_ADDR_ACK_1: begin
                               if (i2c_scl == 0 && counter == START_HOLD) begin 
                                  sda_tristate <= 1;  
                                  //sda_data     <= 1;
                               end
                    
                                if (phase_tick) begin 
                                    i2c_scl <= ~i2c_scl;
                                    
                                    if (i2c_scl == 1) begin
                                        state <= DATA;
                                    end
                                end
                            end 
                
                
                DATA: begin   
                            shift_reg_out <= data_in;  
                                                                                                                                                                         
                            if (i2c_scl == 0 && counter == START_HOLD) begin                                              
                                sda_tristate <= 0;      
                                sda_data     <= shift_reg_out[7 - bit_count];                                   
                            end
                            
                            if (phase_tick) begin
                                i2c_scl <= ~i2c_scl;
                                
                                if (i2c_scl == 1) begin
                                    if (bit_count == 7) begin                 
                                        bit_count <= 0;
                                        state <= DATA_ACK;  
                                    end else begin
                                        bit_count <= bit_count + 1;
                                    end  
                                end
                            end
                        end 
                               
                               
                DATA_ACK: begin
                           if (i2c_scl == 0 && counter == START_HOLD) begin   
                                sda_tristate <= 1;  
                                 
                            end
                            if (phase_tick) begin 
                                i2c_scl <= ~i2c_scl;
                                
                                if (i2c_scl == 1) begin
                                    state <= STOP;
                                end
                            end
                        end  
                               
                    
                 STOP: begin                       
                        if (i2c_scl == 0 && counter == START_HOLD) begin   
                            sda_tristate <= 0;  
                            sda_data     <= 0;  
                        end   
                           
                                                                                   
                        if (phase_tick) begin 
                            i2c_scl <= ~i2c_scl;
                            
                            if (i2c_scl == 1) begin 
                                i2c_scl      <= 1;
                                wait_counter <= 0;                    
                                state <= WAIT;
                            end
                        end                        
                    end       
               
                
                WAIT: begin                                    
                      sda_tristate <= 1;                     
                      if (i2c_scl == 1) begin                                                        
                      wait_counter <= wait_counter + 1;
                      if (wait_counter == WAIT_5MS) begin
                          wait_counter <= 0; 
                          state        <= IDLE;                             
                      end
                  end
              end
                        
                        
                        
                        
                
                //############################//
               //########### READ ###########//
              //#############################//
              
              
              
              
              
                  START_AGAIN: begin                                                                                                                            
                                 bit_count     <= 0;
                                 shift_reg_out <= {addr_1, rw_1};
                        
                                if (counter == START_HOLD) begin
                                    sda_tristate <= 0;  
                                    sda_data     <= 0;  
                                    i2c_scl      <= 1;  
                                end
                
                                if (phase_tick) begin
                                    i2c_scl <= ~i2c_scl;  
                                    
                                    if (i2c_scl == 1) begin  
                                        state <= ADDRESS_1;
                                    end
                                end                                                                                                                        
                            end  
                                               
                
                ADDRESS_1: begin
                        if (i2c_scl == 0 && counter == START_HOLD) begin                                              
                            sda_tristate <= 0;      
                            sda_data     <= shift_reg_out[7 - bit_count];                                   
                        end
                    
                        if (phase_tick) begin
                            i2c_scl <= ~i2c_scl;
                            
                            if (i2c_scl == 1) begin
                                if (bit_count == 7) begin                 
                                    bit_count <= 0;
                                    state <= ADDR_ACK_1;  
                                end else begin
                                    bit_count <= bit_count + 1;
                                end  
                            end
                        end
                    end 
                              
                ADDR_ACK_1: begin
                            if (i2c_scl == 0 && counter == START_HOLD) begin
                                sda_tristate <= 1;            
                            end
                    
                            if (phase_tick) begin 
                                i2c_scl <= ~i2c_scl;
                                
                                if (i2c_scl == 1) begin
                                    state <= READ_REG_ADDR;
                                end
                            end
                        end    
                        
           READ_REG_ADDR: begin   
                        shift_reg_out <=  read_addr_byte;                                                                                                                                              
                        if (i2c_scl == 0 && counter == START_HOLD) begin                                              
                            sda_tristate <= 0;      
                            sda_data     <= shift_reg_out[7 - bit_count];                                   
                        end
                        
                        if (phase_tick) begin
                            i2c_scl <= ~i2c_scl;
                            
                            if (i2c_scl == 1) begin
                                if (bit_count == 7) begin                 
                                    bit_count <= 0;
                                    state <= READ_REG_ADDR_ACK;  
                                end else begin
                                    bit_count <= bit_count + 1;
                                end  
                            end
                        end 
                    end  
                
                
             READ_REG_ADDR_ACK: begin
                              if (i2c_scl == 0 && counter == START_HOLD) begin 
                                  sda_tristate <= 1;                                    
                              end
                    
                            if (phase_tick) begin 
                                i2c_scl <= ~i2c_scl;
                                
                                if (i2c_scl == 1) begin
                                    state <= READ_REG_ADDR_1;
                                end
                            end
                        end   
                        
                
            READ_REG_ADDR_1: begin  
                               shift_reg_out <= read_addr_byte_1;                                                                                                                                               
                            if (i2c_scl == 0 && counter == START_HOLD) begin                                              
                                sda_tristate <= 0;      
                                sda_data     <= shift_reg_out[7 - bit_count];                                   
                             end
                    
                            if (phase_tick) begin
                                i2c_scl <= ~i2c_scl;
                                
                                if (i2c_scl == 1) begin
                                    if (bit_count == 7) begin                 
                                        bit_count <= 0;
                                        state <= READ_REG_ADDR_ACK_1;  
                                    end else begin
                                        bit_count <= bit_count + 1;
                                    end  
                                end
                            end
                        end  
                        
                
            READ_REG_ADDR_ACK_1: begin
                               if (i2c_scl == 0 && counter == START_HOLD) begin 
                                  sda_tristate <= 1;  
                                  //sda_data     <= 1;
                               end
                    
                                if (phase_tick) begin 
                                    i2c_scl <= ~i2c_scl;
                                    
                                    if (i2c_scl == 1) begin
                                        state <= RE_START;
                                    end
                                end
                            end 
                                            
                            
             RE_START: begin     
                       bit_count <= 0;
                       shift_reg_out <= {read_addr, read_rw};
       
                        if (i2c_scl == 0 && counter == START_HOLD) begin
                            sda_tristate <= 0;
                            sda_data     <= 1;  
                        end
                         
                        if (i2c_scl == 1 && counter == START_HOLD) begin
                            sda_tristate <= 0;
                            sda_data     <= 0;  
                        end
                        
                        if (phase_tick) begin
                            i2c_scl <= ~i2c_scl;
                            
                            if (i2c_scl == 1) begin  
                                state <= READ_ADDRESS_1;
                            end
                        end
                    end
                            
           READ_ADDRESS_1: begin                             
                        if (i2c_scl == 0 && counter == START_HOLD) begin                                              
                            sda_tristate <= 0;      
                            sda_data     <= shift_reg_out[7 - bit_count];                                   
                        end
                    
                        if (phase_tick) begin
                            i2c_scl <= ~i2c_scl;
                            
                            if (i2c_scl == 1) begin
                                if (bit_count == 7) begin                 
                                    bit_count <= 0;
                                    state <= READ_ADDR_ACK_1;  
                                end else begin
                                    bit_count <= bit_count + 1;
                                end  
                            end
                        end
                    end 
                    
                              
            READ_ADDR_ACK_1: begin
                            if (i2c_scl == 0 && counter == START_HOLD) begin
                                sda_tristate <= 1;            
                            end
                    
                            if (phase_tick) begin 
                                i2c_scl <= ~i2c_scl;
                                
                                if (i2c_scl == 1) begin
                                    state <= READ_DATA;
                                end
                            end
                        end    
                
               
                
                READ_DATA: begin
                            sda_tristate <= 1'b1;

                            if (phase_tick && i2c_scl == 0) begin        
                                i2c_scl <= 1;                           
                            end else if (phase_tick && i2c_scl == 1) begin
                                        shift_reg_in[7 - bit_count] <= i2c_sda_i;
                                        i2c_scl <= 0;

                            if (bit_count == 8) begin
                                bit_count <= 0;
                                state <= READ_NACK;
                           end else begin
                               bit_count <= bit_count + 1;
                          end
                      end
                  end
                
                                       
                
                READ_NACK: begin                                
                            if (i2c_scl == 0 && counter == START_HOLD) begin
                                sda_tristate <= 0;
                                sda_data     <= 1;  
                            end
                    
                            if (phase_tick) begin
                                i2c_scl <= ~i2c_scl;
                                
                                if (i2c_scl == 1) begin
                                    state <= STOP_1;
                                end
                            end
                        end
                                                                     
                    
               STOP_1:begin                       
                        if (i2c_scl == 0 && counter == START_HOLD) begin   
                            sda_tristate <= 0;  
                            sda_data     <= 0;  
                        end   
                                                                                                              
                        if (phase_tick) begin 
                            i2c_scl <= ~i2c_scl;
                            
                            if (i2c_scl == 1) begin 
                                i2c_scl      <= 1;
                                 wait_counter <= 0; 
                                 sda_tristate <= 1;                   
                                state <= STORE_STATUS_CPTURE_ID;
                            end
                        end                        
                    end    
                
                
                STORE_STATUS_CPTURE_ID: begin  
                                                                                                                                                                   
                                        data_out <= shift_reg_in;
                                                                         
                                   end
                                                      
                default: state <= IDLE;
                
            endcase
        end
    end                                                                                                                                                                                        
               
endmodule
