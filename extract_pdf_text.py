#!/usr/bin/env python3
"""
PDF Text Extraction Script
Extracts all text from the STM32L432KC datasheet PDF and saves it to a text file.
"""

import sys
import os
try:
    import PyPDF2
except ImportError:
    print("PyPDF2 not found. Installing...")
    os.system("pip install PyPDF2")
    import PyPDF2

def extract_pdf_text(pdf_path, output_path):
    """
    Extract text from PDF and save to text file
    
    Args:
        pdf_path (str): Path to the input PDF file
        output_path (str): Path to the output text file
    """
    
    try:
        # Open the PDF file
        with open(pdf_path, 'rb') as pdf_file:
            # Create PDF reader object
            pdf_reader = PyPDF2.PdfReader(pdf_file)
            
            # Get number of pages
            num_pages = len(pdf_reader.pages)
            print(f"Processing {num_pages} pages from {os.path.basename(pdf_path)}...")
            
            # Extract text from all pages
            extracted_text = []
            
            for page_num in range(num_pages):
                try:
                    page = pdf_reader.pages[page_num]
                    text = page.extract_text()
                    
                    # Add page separator
                    extracted_text.append(f"\n{'='*80}\n")
                    extracted_text.append(f"PAGE {page_num + 1} of {num_pages}\n")
                    extracted_text.append(f"{'='*80}\n\n")
                    extracted_text.append(text)
                    extracted_text.append(f"\n\n")
                    
                    # Progress indicator
                    if (page_num + 1) % 10 == 0:
                        print(f"Processed {page_num + 1}/{num_pages} pages...")
                        
                except Exception as e:
                    print(f"Error processing page {page_num + 1}: {e}")
                    extracted_text.append(f"\n[ERROR: Could not extract text from page {page_num + 1}]\n")
            
            # Write extracted text to file
            with open(output_path, 'w', encoding='utf-8') as output_file:
                output_file.writelines(extracted_text)
            
            print(f"\nText extraction completed!")
            print(f"Output saved to: {output_path}")
            print(f"Total pages processed: {num_pages}")
            
            # Display file size
            file_size = os.path.getsize(output_path)
            print(f"Output file size: {file_size:,} bytes ({file_size/1024:.1f} KB)")
            
    except FileNotFoundError:
        print(f"Error: PDF file not found at {pdf_path}")
        print("Please check the file path and try again.")
    except Exception as e:
        print(f"Error processing PDF: {e}")

def main():
    """Main function"""
    
    # Check command line arguments for different PDF files
    if len(sys.argv) > 1:
        pdf_filename = sys.argv[1]
        if pdf_filename == "bq76952":
            pdf_path = "bq76952.pdf"
            output_path = "bq76952_datasheet_text.txt"
        else:
            pdf_path = pdf_filename
            output_path = pdf_filename.replace('.pdf', '_datasheet_text.txt')
    else:
        # Default to STM32 datasheet
        pdf_path = "stm32l432kc-1.pdf"
        output_path = "stm32l432kc_datasheet_text.txt"
    
    # Check if PDF exists
    if not os.path.exists(pdf_path):
        print(f"PDF file '{pdf_path}' not found in current directory.")
        print("Available files:")
        for file in os.listdir("."):
            if file.lower().endswith('.pdf'):
                print(f"  - {file}")
        return
    
    print("PDF Text Extraction Tool")
    print("="*50)
    print(f"Input PDF: {pdf_path}")
    print(f"Output TXT: {output_path}")
    print("="*50)
    
    # Extract text
    extract_pdf_text(pdf_path, output_path)
    
    print("\nDone! You can now read the extracted text from the output file.")

if __name__ == "__main__":
    main()
