from django.shortcuts import render

# Create your views here.

def home(request):
    return render(request, 'home.html', {})

def fac_loc(request):
    return render(request, 'fac_loc.html', {})

def req_lib(request):
    return render(request, 'req_lib.html', {})

def faq(request):
    return render(request, 'faq.html', {})