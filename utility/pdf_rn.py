import requests
import urllib
from bs4 import BeautifulSoup
import os

url="https://www.thebiomics.com/blog/springer-free-books-download.html?fbclid=IwAR2YRaJyUMEBkstYZrXPLx4iActO7TTwvepSYBJzAAAxhJji1PmVLpdRj_M"

req = urllib.request.Request(url,  headers={'User-Agent' : "Magic Browser"})
html = urllib.request.urlopen( req )
soup = BeautifulSoup(html)
tags = soup.findAll("a", {"class": "download-link link-external link m-0"})

links_table = soup.select("table")[0]
table_rows = links_table.select("tr")

link2book = {}

num = len(table_rows)
count = 0
for row in table_rows:
    count += 1
    if count < 2:
        continue
    # dl_url = tag.get("download")
    entries = row.find_all('td')

    title = entries[0].find('a')
    if (title is None):
        print("Invalid title")
        continue
    title = title.getText()

    dl_url = entries[1].find('a')
    if (dl_url is None):
        print("Invalid dl link")
        print(row)
        continue
    dl_url = dl_url.get("download")
    
    pre = "https://link.springer.com/content/pdf/"
    dl_name = dl_url[len(pre):]

    link2book[dl_name] = title


pdfFolder = "/media/pieter/S10HDD/Springer Books/"

for fname in os.listdir(pdfFolder):
    if not fname.startswith("10"):
        continue

    # print("Renaming", fname)

    if not fname in link2book:
        print("Could not find title for", fname)
        continue

    title = link2book[fname]
    # print(title)

    if "/" in title:
        title = title.replace("/", " or ")

    if not os.path.isfile(pdfFolder+fname):
        print(pdfFolder+fname)

    os.rename(pdfFolder+fname, pdfFolder+title+".pdf")
